# MBlocks-MB

## Cube to cube command API

SEND <msgid> <id> <message>     :       sends a message; receiving cubes that don't match id ignore it
SENDCMD <msgid> <id> <command>  :       sends a command; receiving cubes that don't match id ignore it
ACK <msgid>                     :       acknowledges receipt of a message (only for SEND/SENDCMD messages)
BDCAST <msgid> <message>        :       broadcasts a message on all faces
BDCASTCMD <msgid> <command>     :       broadcasts a command on all faces (command will be executed by all cubes that receive it)

<msgid> is formed by appending an integer count to the end of the sender's MAC address. Example: C6:EA:B8:01:3D:EE+12. The MAC addresses are hashed on each cube.

Note that in implementation, words are semicolon separated as commands can contain spaces.