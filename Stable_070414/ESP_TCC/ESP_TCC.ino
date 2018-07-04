#include "./serverFunc.h"

void setup(void){
  startESP();
}

void loop(void){
  handleServer();
  handleMaster();
}
