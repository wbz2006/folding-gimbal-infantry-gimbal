#include <assert.h>
#include <stdio.h>
#include "message_center.h"
int main(void)
{
    Subscriber_t *sub = SubRegister("order", sizeof(int));
    Publisher_t *pub = PubRegister("order", sizeof(int));
    int out = -1;
    assert(!SubGetMessage(sub, &out));
    assert(out == -1);
    for (int round = 0; round < 30; ++round)
    {
        for (int i = 0; i < QUEUE_SIZE; ++i)
        {
            int value = round * 100 + i;
            PubPushMessage(pub, &value);
        }
        for (int i = 0; i < QUEUE_SIZE; ++i)
        {
            assert(SubGetMessage(sub, &out));
            assert(out == round * 100 + i);
        }
        assert(!SubGetMessage(sub, &out));
    }
    for (int i = 0; i < QUEUE_SIZE + 3; ++i)
        PubPushMessage(pub, &i);
    for (int i = 3; i < QUEUE_SIZE + 3; ++i)
    {
        assert(SubGetMessage(sub, &out));
        assert(out == i);
    }
    puts("PASS queue FIFO, wraparound, overflow, empty");
    return 0;
}
