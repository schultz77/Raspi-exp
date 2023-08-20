from gtts import gTTS
from playsound import playsound

language = 'pt-br'
tts = gTTS(text="Ola Sonia, tudo bem? Como voce dormiu hoje? O que voce vai fazer hoje?",
           lang=language,
           slow=False)

tts.save('./tts.mp3')
playsound('./tts.mp3')