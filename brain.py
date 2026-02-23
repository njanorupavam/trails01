import cv2
import serial
import time
import google.generativeai as genai
from PIL import Image
import io
import chess
import chess.engine

# --- CONFIG ---
GEMINI_API_KEY = "AlzaSyBE6hDo-PiJNp_2GHOMDfFq_FkLzkY07mg"  # Your Gemini API key
CAMERA_URL = "http://10.81.34.95:8080/video"
COM_PORT = r'\\.\COM15'

# Configure Gemini
genai.configure(api_key=GEMINI_API_KEY)

def analyze_board_with_gemini(image_path):
    """Use Gemini Vision to analyze the chess board"""
    try:
        # Initialize Gemini model
        model = genai.GenerativeModel('gemini-1.5-flash')  # Using flash for faster response
        
        # Load and prepare image
        img = Image.open(image_path)
        
        # Prompt for Gemini
        prompt = """You are a chess grandmaster. Look at this chess board image and:
        1. First, output the FEN notation of the current position
        2. Then, output the best move for the player to move in UCI format (e.g., e2e4)
        
        Format your response exactly like this:
        FEN: <fen_string>
        MOVE: <uci_move>
        
        Only output these two lines, nothing else."""
        
        # Get response from Gemini
        response = model.generate_content([prompt, img])
        
        # Parse response
        text = response.text.strip()
        print(f"Gemini response:\n{text}")
        
        fen_line = [line for line in text.split('\n') if line.startswith('FEN:')][0]
        move_line = [line for line in text.split('\n') if line.startswith('MOVE:')][0]
        
        fen = fen_line.replace('FEN:', '').strip()
        move = move_line.replace('MOVE:', '').strip().lower()
        
        return fen, move
        
    except Exception as e:
        print(f"Gemini API error: {e}")
        return None, None

def validate_uci_move(move):
    """Validate UCI move format"""
    if not move or len(move) < 4:
        return False
    if move[0] not in 'abcdefgh' or move[2] not in 'abcdefgh':
        return False
    if not move[1].isdigit() or not move[3].isdigit():
        return False
    return True

def square_to_coordinates(square):
    """Convert chess square to robot coordinates"""
    # CALIBRATE THESE VALUES FOR YOUR ROBOT!
    files = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}
    ranks = {'1': 0, '2': 1, '3': 2, '4': 3, '5': 4, '6': 5, '7': 6, '8': 7}
    
    # Adjust these based on your physical setup
    square_size = 35  # mm per square
    start_x = 100     # Starting X coordinate for a1
    start_y = 100     # Starting Y coordinate for a1
    
    file_idx = files[square[0]]
    rank_idx = ranks[square[1]]
    
    # Uncomment if your board orientation is reversed
    # rank_idx = 7 - rank_idx
    
    x = start_x + (file_idx * square_size)
    y = start_y + (rank_idx * square_size)
    
    return x, y

def capture_board_image():
    """Capture image from camera"""
    cap = cv2.VideoCapture(CAMERA_URL)
    
    # Let camera warm up
    for _ in range(5):
        cap.read()
        time.sleep(0.1)
    
    ret, frame = cap.read()
    if ret:
        # Save high quality image
        cv2.imwrite("board_current.jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        print("✓ Board image captured")
    else:
        print("✗ Failed to capture image")
    
    cap.release()
    return ret

def get_engine_move_fallback(fen):
    """Fallback to Stockfish if Gemini fails"""
    try:
        engine = chess.engine.SimpleEngine.popen_uci("stockfish.exe")
        board = chess.Board(fen)
        result = engine.play(board, chess.engine.Limit(time=1.0))
        engine.quit()
        return result.move.uci()
    except:
        return "e2e4"  # Ultra fallback

# --- Main execution ---
print("="*50)
print("CHESS ROBOT CONTROLLER")
print("="*50)

# Connect to Arduino/Pico
print(f"\nConnecting to robot on {COM_PORT}...")
try:
    ser = serial.Serial(COM_PORT, 115200, timeout=3)
    time.sleep(2)  # Wait for Arduino reset
    print("✓ Robot connected")
except Exception as e:
    print(f"✗ Failed to connect: {e}")
    exit()

# Main loop - you can add a loop for multiple moves
while True:
    print("\n" + "="*50)
    
    # Capture board
    if not capture_board_image():
        print("Failed to capture board, retrying...")
        time.sleep(1)
        continue
    
    # Analyze with Gemini
    print("Analyzing with Gemini...")
    fen, best_move = analyze_board_with_gemini("board_current.jpg")
    
    if not best_move or not validate_uci_move(best_move):
        print("Gemini analysis failed, using fallback...")
        if fen:
            best_move = get_engine_move_fallback(fen)
        else:
            best_move = "e2e4"
    
    print(f"\n✓ Best move: {best_move}")
    
    # Convert to coordinates
    from_square = best_move[:2]
    to_square = best_move[2:4]
    
    start_x, start_y = square_to_coordinates(from_square)
    end_x, end_y = square_to_coordinates(to_square)
    
    # Heights (adjust based on your robot)
    pick_height = 20
    place_height = 20
    
    # Send command
    command = f"MOVE {start_x},{start_y},{pick_height} {end_x},{end_y},{place_height}\n"
    print(f"Sending: {command.strip()}")
    ser.write(command.encode())
    
    # Wait for response
    response = ser.readline().decode().strip()
    print(f"Robot: {response}")
    
    # Ask if user wants to continue
    again = input("\nMake another move? (y/n): ").lower()
    if again != 'y':
        break

ser.close()
print("\n✓ Robot session ended")