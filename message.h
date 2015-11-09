#ifndef MESSAGE_H_
#define MESSAGE_H_

// void message_init(void);
void message_deinit(void);

void push_message(int faceNum, char *txData);
void process_message(char *msg);
bool duplicate(char *msgid);

#endif /* MESSAGE_H_ */