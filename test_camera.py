import cv2

# --- REPLACE WITH THE IP SHOWN ON YOUR PHONE ---
# Use the full URL provided by the Iriun app
ip_url = "https://10.81.34.95:8080/video" 

def capture_from_phone():
    # We pass the URL string instead of index 0
    cap = cv2.VideoCapture(ip_url)
    
    if not cap.isOpened():
        print("Error: Could not connect to Iriun. Check your Phone IP!")
        return None

    ret, frame = cap.read()
    if ret:
        cv2.imwrite("phone_shot.jpg", frame)
        print("Captured image from phone successfully!")
    
    cap.release()
    return "phone_shot.jpg"

# Test it
capture_from_phone()