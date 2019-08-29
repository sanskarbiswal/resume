//Basic Setup
int votes[2] = {0,0};
int total = 9;
bool flag = true;
int voted = 0;
// make an array to save Sev Seg pin configuration of numbers

int num_array[10][7] = {  { 1,1,1,1,1,1,0 },    // 0
                          { 0,1,1,0,0,0,0 },    // 1
                          { 1,1,0,1,1,0,1 },    // 2
                          { 1,1,1,1,0,0,1 },    // 3
                          { 0,1,1,0,0,1,1 },    // 4
                          { 1,0,1,1,0,1,1 },    // 5
                          { 1,0,1,1,1,1,1 },    // 6
                          { 1,1,1,0,0,0,0 },    // 7
                          { 1,1,1,1,1,1,1 },    // 8
                          { 1,1,1,0,0,1,1 }};   // 9
//function header
void Num_Write(int);
                         
void setup() {
  // put your setup code here, to run once:
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  // set pin modes
  pinMode(2, OUTPUT);   
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  Num_Write(0);
  Serial.begin(9600);
  Serial.println("Start Polls");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(flag){
    if(digitalRead(A0) == LOW){
      votes[0] = votes[0]+1;
      Serial.print("Candidate 1: "); Serial.print(votes[0]); Serial.println("");
      voted = voted+1;
      digitalWrite(10, HIGH);
      Num_Write(votes[0]);
      break;
    }
    else if(digitalRead(A1) == LOW){
      votes[1] = votes[1]+1;
      Serial.print("Candidate 2: "); Serial.print(votes[1]); Serial.println("");
      voted = voted+1;
      digitalWrite(11, HIGH);
      Num_Write(votes[1]);
      break;
    }
  }
  if(voted > 9) {flag = false;}
  if(flag == false && voted ==10) {voted = voted+1; limit();}
  delay(1000);
}

// this functions writes values to the sev seg pins  
void Num_Write(int number) 
{
  int pin= 2;
  for (int j=0; j < 7; j++) {
   digitalWrite(pin, num_array[number][j]);
   pin++;
  }
}
void limit(){
  Serial.println("Max. No.of Voters for this contituency reached. Voting is now closed");
}
