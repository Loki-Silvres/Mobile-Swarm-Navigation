
# Python Chatbot

## Overview
This Python chatbot operates using a zero-shot prompt approach, enabling it to execute tasks without prior fine-tuning or additional training. User input is processed to extract intents, which then trigger specific nodes to perform the requested tasks. The chatbot runs on a local Flask server and interacts with users via a web interface.

---

## How to Run

### Step 1: Start the Chatbot
Run the following command in your terminal:
```bash
python3 path_to_app.py
```

### Step 2: Check Terminal Output
If there are no errors, the terminal should display the following messages:
```
* Serving Flask app 'app'
* Debug mode: on
WARNING: This is a development server. Do not use it in a production deployment. Use a production WSGI server instead.
* Running on http://127.0.0.1:5000
  Press CTRL+C to quit
* Restarting with stat
* Debugger is active!
* Debugger PIN: 292-188-787
```

### Step 3: Access the Web Interface
Open your browser and navigate to:
```
http://127.0.0.1:5000
```

---

## Chatbot Features

### Zero-Shot Prompt Functionality
The chatbot uses a zero-shot prompting technique, where the model performs tasks without specific training. Responses are filtered to extract intents in a structured format. Based on the extracted intent, appropriate nodes are triggered to execute the task.

### Adjustable Parameters
Users can customize chatbot responses with the following parameters:

#### 1. **Temperature**
Controls the randomness of the output.
- **Range:** [0, 2]
- **Guidelines:**
  - `0 < Temperature < 0.3`: Focused and predictable responses.
    - **Use Case:** Clear, concise answers or strict formats.
  - `0.4 < Temperature < 1`: Balanced creativity and predictability.
  - `1.1 < Temperature < 2`: Highly creative and diverse responses.

#### 2. **Top-p**
Regulates the diversity of the output using nucleus sampling.
- **Range:** 0 < Top-p ≤ 1
- **Guidelines:**
  - `0.0 < Top-p < 0.5`: Deterministic and safe outputs.
  - `0.6 < Top-p < 0.9`: Balanced creativity and coherence.
  - `Top-p > 0.9`: Diverse and innovative outputs.

#### 3. **Max Length**
Defines the maximum number of tokens the model can generate in a response.

---

## Intent Extraction and Task Execution

### Approach

The chatbot processes user requests with the following methodology:

1. **Zero-Shot Prompt Template:**
   ```plaintext
   give intents with every response in the form of @#intent#@
   for manipulation tasks, intent is @#manipulation#@, and the object to be manipulated is marked with #object.
   ```
   **Example:**
   - **User Input:**  
     `Please bring me a bottle.`
   - **Model Response:**  
     `Your request is being processed. #bottle`

2. **Processing Workflow:**
   - Generate chatbot content using:
     ```python
     model.generate_content([[zero_shot_prompt, user_message]])
     ```
   - Use Python regular expressions (`re`) to:
     - Extract intents in the format `@#intent#@`.
     - Identify objects to manipulate (e.g., `#object`).
   - Trigger the appropriate process using `os.system()` to pass the parameters to the node.

---

## Example Workflow

### Input:
```plaintext
Please bring me a bottle.
```

### Zero-Shot Prompt Response:
```plaintext
Your request is being processed. #bottle
```

### Intent Extraction:
- Intent: `@#manipulation#@`
- Object: `#bottle`

### Execution:
Launches a terminal process to handle the manipulation task and passes the object (`bottle`) as a parameter.

---

## Important Notes

1. **Internet and API Key Requirements**
   - The chatbot requires an active internet connection and a valid API key for its standard operation.

2. **Development Server Warning**
   - The Flask server is intended for development purposes. Avoid using it for production deployment. Use a production-grade WSGI server instead.
