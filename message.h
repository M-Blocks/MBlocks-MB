#ifndef MESSAGE_H_
#define MESSAGE_H_

// void message_init(void);
void message_deinit(void);

void push_message(char *txData);
void prepare_message_send(const char *type, int msgCnt, char *destID, char *msg);
void prepare_message_bdcast(const char *type, int msgCnt, char *msg);
void process_message(char *msg);

#endif /* MESSAGE_H_ */