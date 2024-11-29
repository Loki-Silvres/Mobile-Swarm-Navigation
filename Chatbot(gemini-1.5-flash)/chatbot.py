import google.generativeai as genai
from IPython.display import HTML, Markdown, display
genai.configure(api_key="AIzaSyBrqbxl9HvXlW__1gG5S9Yl1K549D4vKpA")
import os
import re
from google.api_core import retry

high_temp_model = genai.GenerativeModel(
    'gemini-1.5-flash',
    generation_config=genai.GenerationConfig(temperature=2.0))


# When running lots of queries, it's a good practice to use a retry policy so your code
# automatically retries when hitting Resource Exhausted (quota limit) errors.
retry_policy = {
    "retry": retry.Retry(predicate=retry.if_transient_error, initial=10, multiplier=1.5, timeout=300)
}

def removedec(text):
    pattern = r"<code>(.*?)</code>|@#(.*?)#@"
    cleaned_text = re.sub(pattern, "", text)
    print(cleaned_text)
    
def operation(text):
    pattern = r"@#(.*?)#@"
    intent = re.findall(pattern, text)
    if (len(intent) == 0 ): 
        return
    if (intent[0] == "code"):
        pattern = r"<code>(.*?)</code>"
        matches = re.findall(pattern, text, re.DOTALL)
        os.system(f"gnome-terminal -- bash -c '{matches[0]}'")
    
model = genai.GenerativeModel(
    'gemini-1.5-flash-001',
    generation_config=genai.GenerationConfig(
        temperature=0.1,
        top_p=1,
        max_output_tokens=100,
    ))

zero_shot_prompt = """
You are a chatbot appointed at a factory for managing a swarm of robots.  and in every response you have to display intent in form of @#intent#@, and code in form <code></code>
Your responsibilities include:


1. Responding to user inquiries.
2. Spawning more robots or starting the simulation
   if user didn't give no of bots print enter no of bots and no codespawnS,
   if he gives then only display code and give a message.
   code is
   cd ~/ros2_ws3 then source install/setup.bash then python3 /home/vc940/ros2_ws3/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch/map_maker_launch.py --num_bots = {number_of_bots}'''
   seperate the code with &&
   display above in format 'code' in same line
   intent is @#code#@

3. when someone asked to do task from one of navigation,manipulation:-
   give confirmation that your task is been processed.as the intent will be processed
   intent is @#task#@



   
Behave like a helpful, professional chatbot and ensure concise and accurate responses.

 """
while(1):
    s = input("user:")
    response = model.generate_content([zero_shot_prompt,s], request_options=retry_policy)
    text = response.text
    removedec(text)
    operation(text)
      