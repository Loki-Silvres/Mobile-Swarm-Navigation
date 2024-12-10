from flask import Flask, render_template, request, jsonify
import google.generativeai as genai
import os
import re
from google.api_core import retry
import subprocess
import markdown
app = Flask(__name__)
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
zero_shot_prompt = """
You are a chatbot appointed  for managing a swarm of robots.  and in every response you have to display intent in form of @#intent#@, and code in form <code></code>

Your responsibilities include:

 Responding to user inquiries.


1. if someone ask to bring some object ,go to some object or find some object:-
   give confirmation to user and  $object 
   if multiple objects give confirmation to user then $object1,$object2 and $object3.
   user:[take me to a bucket,chair]
   response:[i am moving to bucket and chair. $bucket $chair]
   intent is @#manipulation#@


2. start autonomous mapping or exploration
   give confirmation to user and code 
   code is cd ~/53_m2_kalyani && ros2 launch ~/53_m2_kalyani/Team_53_ws/auto_explore/launch/auto_map.launch.py
   and intent is @#mapping#@

3. if user asks to save the map:-
    then display map has been saved.
    else display map has not been saved.
    intent is @#mapsaving#@


   
Behave like a helpful, professional chatbot and ensure concise and accurate responses.

 """
def removedec(text):
    pattern = r"<code>(.?)</code>|@#(.*?)#@|!\S+|\$\w+|<code>(.*?)</code>"
    cleaned_text = re.sub(pattern, "", text)
    return cleaned_text

  
def operation(text):
    pattern = r"@#(.*?)#@"
    intent = re.findall(pattern, text)
    if (len(intent) == 0 ): 
        return
 
    if(intent[0]=="mapping"):
        pattern = r"<code>(.*?)</code>"
        matches = re.findall(pattern, text, re.DOTALL)
        for i in matches:
            print(i)
            os.system(f'gnome-terminal -- bash -c "{i}; exec bash"')
    elif(intent[0]=="manipulation"):
        pattern = r'\$\w+'
        hashtags = re.findall(pattern, text,re.DOTALL)
        print(hashtags)
        for i in hashtags:
            print(i)
            os.system(f"gnome-terminal -- bash -c 'ros2 topic pub -1 /get_object std_msgs/String \"data: \\\"{i[1:]}\\\"\"'")
    elif(intent[0]=="mapsaving"):
           os.system(f"gnome-terminal -- bash -c 'ros2 run nav2_map_server map_saver_cli -f ~/53_m2_kalyani/Team_53_ws/auto_explore/maps/map_new'")
            
    
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
    operation(raw_text)  
    return jsonify({"response": cleaned_text})

if __name__ == '__main__':
    app.run(debug=True)
