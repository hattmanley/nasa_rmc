void setup() {
  // put your setup code here, to run once:
  int pwmfreq = 250;   //250hz = 4ms each
analogWriteFrequency(2, pwmfreq);
analogWriteFrequency(3, pwmfreq);
analogWriteFrequency(4, pwmfreq);
analogWriteFrequency(5, pwmfreq);
analogWriteFrequency(6, pwmfreq);
analogWriteFrequency(7, pwmfreq);
analogWriteResolution(12); //0 - 4095, at 250hz, 4ms is split into 4096 parts, 1~2ms is from 1023-2047




}

void loop() {
  int testfreq = 1537;
  // put your main code here, to run repeatedly:
analogWrite(2,testfreq);
analogWrite(3,testfreq);
analogWrite(4,testfreq);
analogWrite(5,testfreq);
analogWrite(6,testfreq);
analogWrite(7,testfreq);



}
