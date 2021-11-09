#Programa que convierte voz a texto
import speech_recognition as sr
r = sr.Recognizer()

with sr.Microphone() as source:
    print("Estoy Escuchando...")
    audio = r.listen(source)

    try:
        text = r.recognize_google(audio, language='es-ES')
        print("Lo que escuché fue:",str(text))
    except:
        print("No te he entendido")
