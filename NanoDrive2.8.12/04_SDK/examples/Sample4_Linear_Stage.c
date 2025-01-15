#include <stdlib.h>
#include <stdio.h>
#include "NTControl.h"
#include <Windows.h>
int main(int argc, char* argv[])
{
    unsigned int status;
    int position;
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

    while (1)
    {
        /*X channel is linearly available*/
        /*Set the relative displacement of channel X to move forward and backward,
        and obtain the position and status after the displacement is completed*/
        NT_GotoPositionRelative_S(ntHandle, channel, 100000);
        Sleep(1000);
        NT_GetPosition_S(ntHandle, channel, &position);
        NT_GetStatus_S(ntHandle, channel, &status);
        printf("position = %d£¬status = %d\r\n", position, status);


        NT_GotoPositionRelative_S(ntHandle, channel, -100000);
        Sleep(1000);
        NT_GetPosition_S(ntHandle, channel, &position);
        NT_GetStatus_S(ntHandle, channel, &status);
        printf("position = %d£¬status = %d\r\n", position, status);


    }
    NT_CloseSystem(ntHandle);

    return 0;
}
