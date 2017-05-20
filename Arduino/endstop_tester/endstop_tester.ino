
int rotBottom = 33;
int rotTop = 34;
int linBottom = 35;
int linTop=36;


void setup() {
  // put your setup code here, to run once:
  pinMode(33, INPUT); //rotation bottom
  pinMode(34, INPUT); //rotation top
  pinMode(35, INPUT); //linear bottom
  pinMode(36, INPUT); //linear top
  Serial.begin(9600);
  Serial.write("started");
}

void loop() {
  if(digitalRead(rotBottom)){
    Serial.println("Rotation Bottom");
 }
  if(digitalRead(rotTop)){
    Serial.println("Rotation Top");
  }
  if(digitalRead(linBottom)){
    Serial.println("Linear Bottom");
  }
  if(digitalRead(linTop)){
    Serial.println("Linear Top");
  }
}
