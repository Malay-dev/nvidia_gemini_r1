#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import google.generativeai as genai
import re
import ast
from langchain_core.prompts import PromptTemplate

import speech_recognition as sr
import pyttsx3 
from dotenv import load_dotenv
import os

load_dotenv()

api_key = os.getenv("GOOGLE_API_KEY")

if not api_key:
    raise ValueError("API_KEY is missing! Please set it in the .env file.")

def speak_text(command):
    engine = pyttsx3.init()
    engine.say(command)
    engine.runAndWait()

def listen_and_convert(timeout=5, phrase_time_limit=10):
    r = sr.Recognizer()
    
    try:
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source, duration=0.2)
            
            audio = r.listen(source, timeout=timeout, phrase_time_limit=phrase_time_limit)
            recognized_text = r.recognize_google(audio).lower()
            print("Did you say:", recognized_text)
            speak_text(recognized_text)
            return recognized_text
            
    except sr.WaitTimeoutError:
        print("Listening timed out while waiting for phrase to start.")
        return None
        
    except sr.RequestError as e:
        print(f"Could not request results; {e}")
        return None
        
    except sr.UnknownValueError:
        print("Sorry, I did not understand that.")
        return None


def extract_array(output_string):
    array_str = re.search(r'\[\[.*\]\]', output_string).group(0)
    return array_str

def convert_to_array(array_str):
    return ast.literal_eval(array_str)

def main():
    rospy.init_node('motion_instruction_publisher', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    genai.configure(api_key=api_key)
    model = genai.GenerativeModel(model_name="gemini-1.5-flash")

    # Define a PromptTemplate with placeholders for motion instructions
    template = """
    Act as a motion coordinator for velocity commands. Convert the following motion instructions into a precise 2D array of velocities and time.

    Instructions to process: {motion_instruction}

    Important rules:
    1. Default velocity is 10 unit unless specified.
    2. Calculate vx as cos(angle) in decimal and vy as sin(angle) in decimal.
    3. vx and vy represent movement along the x-axis and y-axis, respectively.
    4. Angular velocity in z (ωz): rotation around the z-axis. Convert angles from degrees to radians.
    5. Time (t): if only linear velocity is changed, calculate as distance divided by velocity.
    6. Time (t): if only angular velocity is changed, set as 2π (for a full rotation).
    7. All movements should be relative to the previous position.
    8. Ensure the final sublist is [0, 0, 0, 0] to stop the motion.
    9. Round all numbers to 2 decimal places.

    Examples of valid instructions:
    - "Move forward with velocity 2 for 3 seconds"
    - "Turn 90 degrees clockwise at angular velocity 0.5"
    - "Move diagonally left at 45 degrees for 5 seconds"

    Only respond with the resulting array in this exact format:
    Result: [[vx1, vy1, ωz1, t1], [vx2, vy2, ωz2, t2], ..., [0, 0, 0, 0]]

    """

    # Create a PromptTemplate object
    prompt_template = PromptTemplate(
        input_variables=["motion_instruction"],
        template=template
    )

    while not rospy.is_shutdown():
        # Take user input for the motion instruction
        
        command=input("Enter 1 for text input or 2 for voice input or any other number to exit: ")
        if command=='1':
            motion_instruction = input("Enter your motion instructions as you pressed 1 (or type 'exit' to quit): ")
        
        elif command=='2':
            motion_instruction=listen_and_convert()
        
        else :
            print("Exiting...Enter correct instruction i.e. is 1 or 2 to give input")
            break
        
        

        # Check if the user wants to exit
        if motion_instruction.lower() == 'exit':
            break

        print(motion_instruction)

        final_prompt = prompt_template.format(motion_instruction=motion_instruction)

        response = model.generate_content(final_prompt)

        output = response.text.strip()
        print(output)

        extracted_array = extract_array(output)
        print(extracted_array)

        actual_array = convert_to_array(extracted_array)

        rate = rospy.Rate(10) 

        for i in range(len(actual_array)):
            twist = Twist()
            twist.linear.x = actual_array[i][0]  # vx
            twist.linear.y = actual_array[i][1]  # vy
            twist.angular.z = actual_array[i][2]  # ωz

            cmd_vel_pub.publish(twist)

            time_delay = actual_array[i][3]   
            rospy.sleep(time_delay)  

    rospy.signal_shutdown("Finished executing instructions.")

if __name__ == '__main__':
    main()