from gtts import gTTS


tts = gTTS('hello, my name is turtlebot')

tts.save('hello.mp3')