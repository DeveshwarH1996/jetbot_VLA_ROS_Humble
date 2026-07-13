from fastapi import FastAPI, UploadFile, Form
import uvicorn
import random

app = FastAPI()

# This mock server mimics the VLA API a real model (like OpenVLA) would use.
# It allows us to test the ROS2 pipeline without needing a 24GB GPU active.

@app.post("/predict")
async def predict(image: UploadFile, task: str = Form(...)):
    print(f"Received request: Task='{task}' | Image size: {len(await image.read())} bytes")
    
    # In a real VLA, this would be the result of a transformer-based vision-language model.
    # For simulation testing, we return a few common action tokens.
    actions = ["move_fwd", "turn_l", "turn_r", "stop"]
    selected_action = random.choice(actions)
    
    return {
        "status": "success",
        "actions": [selected_action],
        "confidence": 0.92
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
