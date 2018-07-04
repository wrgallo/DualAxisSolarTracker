/*
 Copyright (C) 2018 Wellington Rodrigo Gallo <w.r.gallo@grad.ufsc.br>
  This program is a free software; 
  You can:
    * redistribute it
    * modify it
  Under the terms of the GNU General Public License
  published by the Free Software Foundation.
*/

/*
  TODO:
  * ESP Server
  * Control Unit Version
*/
#include "myFunctions.h"

void setup() { configThisUnit(); refreshSensorData(); report2Master(); }

void loop()
{
  if( checkTimerFlag() )
  {
    refreshSensorData();
    report2Master();
  }
  delay(1000);
}
