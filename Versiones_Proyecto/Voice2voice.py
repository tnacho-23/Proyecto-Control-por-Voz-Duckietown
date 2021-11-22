import pyttsx3
import speech_recognition as sr
eng=pyttsx3.init()
import win32com
r = sr.Recognizer()

with sr.Microphone() as source:
    print("Estoy Escuchando...")
    audio = r.listen(source)

    try:
        text = r.recognize_google(audio, language='es-ES')
        leer=str(text)
        eng.say(leer) 
        eng.runAndWait()

    except:
        eng.say("No te he entendido")
        eng.runAndWait()




