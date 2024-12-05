from flask import Flask, render_template, request, jsonify
import google.generativeai as genai
import os
import re
from google.api_core import retry
import subprocess

app = Flask(__name__)

# Gemini API setup
genai.configure(api_key="AIzaSyBrqbxl9HvXlW__1gG5S9Yl1K549D4vKpA")
model = genai.GenerativeModel(
    'gemini-1.5-flash-001',
    generation_config=genai.GenerationConfig(
        temperature=0.1,
        top_p=1,
        max_output_tokens=100,
    ))

retry_policy = {
    "retry": retry.Retry(predicate=retry.if_transient_error, initial=10, multiplier=1.5, timeout=300)
}

# Zero-shot prompt for the chatbot
zero_shot_prompt = """

You are a chatbot appointed at a factory for managing a swarm of robots.  and in every response you have to display intent in form of @#intent#@, and code in form <code></code>
Your responsibilities include:


1. Responding to user inquiries.
2. Spawning more robots or starting the simulation
   if user didn't give no of bots print enter no of bots and no codespawnS,
   if he gives then only display code and give a message.
   code is
   cd ~/turtlebot3_ws then source install/setup.bash then python3 ~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch/map_maker_final_new_launch.py --num_bots {number_of_bots}'''
   seperate the code with &&
   display above in format 'code' in same line
   intent is @#code#@
3. start autonomous mapping or exploration
   code is ros2 launch ~/auto_explore_ws/src/auto_explore/launch/auto_map.launch.py
   and intent is @#mapping#@
   
4. if user asks to save map,and autonomous mapping has been executed previously in chat
    code is ros2 run nav2_map_server map_saver_cli -f ~/auto_explore_ws/src/auto_explore/maps/map_saved
    intent is #@@savemap#@


5. when someone asked to do task from one of navigation,manipulation:-
   give confirmation that your task is been processed.as the intent will be processed
   intent is @#task#@



   
Behave like a helpful, professional chatbot and ensure concise and accurate responses.


"""  # (complete prompt here)

def removedec(text):
    pattern = r"<code>(.*?)</code>|@#(.*?)#@|!\S+"
    cleaned_text = re.sub(pattern, "", text)
    return cleaned_text

def operation(text):
    pattern = r"@#(.*?)#@"
    intent = re.findall(pattern, text)
    if not intent:
        return None
    if intent[0] == "code":
        pattern = r"<code>(.*?)</code>"
        matches = re.findall(pattern, text, re.DOTALL)
        if matches:
            os.system(f"gnome-terminal -- bash -c '{matches[0]}'")
    elif intent[0] == "navigation":
        matches = re.findall(r'!\S+', text)
        for i in matches:
            command = ["xterm", "-e", "python3", f"/home/vc940/navigation.py", f"{i}"]
            subprocess.run(command)
    return intent[0]
    
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/get_response', methods=['POST'])
def get_response():
    user_message = request.json.get('message')
    if not user_message:
        return jsonify({"response": "Please provide a message."})

    # Generate response using Gemini
    response = model.generate_content([zero_shot_prompt, user_message], request_options=retry_policy)
    raw_text = response.text

    # Process response
    cleaned_text = removedec(raw_text)
    operation(raw_text)  # Execute code if required

    return jsonify({"response": cleaned_text})

if __name__ == '__main__':
    app.run(debug=True)
