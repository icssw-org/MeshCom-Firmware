#include "msgid_counter.h"

namespace
{

// Folds any integer into 0..kMsgIdMax. Negative values are possible on paper
// (node_msgid is a plain int in both platform structs) and must not reach a
// frame; a modulo alone would keep the sign.
int fold(long long v)
{
    const long long span = (long long)kMsgIdMax + 1;
    long long r = v % span;
    if (r < 0)
    {
        r += span;
    }
    return (int)r;
}

} // namespace

int msgIdAdvance(int current)
{
    return fold((long long)current + 1);
}

bool msgIdNeedsPersist(int msgid)
{
    return fold(msgid) % kMsgIdPersistStep == 0;
}

int msgIdAfterLoad(int stored)
{
    return fold((long long)stored + kMsgIdPersistStep);
}
