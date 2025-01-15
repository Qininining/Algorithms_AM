#include <stdlib.h>
#include <stdio.h>
#include "NTControl.h"
#include <Windows.h>
int main(int argc, char* argv[])
{
    unsigned int status;
    unsigned int angle;
    int revolution;
    unsigned int channel = 0;
    NT_INDEX ntHandle;

    /* When opening a controller you must select one of the two communication
      modes:
   sync: only commands from the set of synchronous commands can
       be used in the program. In sync. communication mode commands like
       GetPosition, GetStatus etc. return the requested value directly.
       this is easier to program, especially for beginners.
   async: only asynchronous commands can be used. In this mode Get...
       commands send a request message to the NPS controller but do not
       wait for the reply. The replied message must be caught with special
       commands ReceiveNextPacket, which are only available in async. communication
       mode. Please read the NPS Programmer's Guide for more information. */

       /* Open the NPS in synchronous communication mode */
    NT_OpenSystem(&ntHandle, "usb:ix:0", "sync");
    /*X channel setting rotation sensor type*/
    NT_SetSensorType_S(ntHandle, channel, RS20);
    while (1)
    {
        /*X channel rotation is available*/
        /*Set the relative displacement of channel X to move forward and backward,
        and obtain the angle and status after the displacement is completed*/
        NT_GotoAngleRelative_S(ntHandle, channel, 5000000,0);
        Sleep(1000);
        NT_GetAngle_S(ntHandle, channel, &angle, &revolution);
        NT_GetStatus_S(ntHandle, channel, &status);
        printf("angle = %d£¬revolution = %d£¬status = %d\r\n", angle, revolution, status);


        NT_GotoAngleRelative_S(ntHandle, channel, -5000000,0);
        Sleep(1000);
        NT_GetAngle_S(ntHandle, channel, &angle, &revolution);
        NT_GetStatus_S(ntHandle, channel, &status);
        printf("angle = %d£¬revolution = %d£¬status = %d\r\n",angle,revolution,status);


    }
    NT_CloseSystem(ntHandle);

    return 0;
}
