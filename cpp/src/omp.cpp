/*
It is more probable that earlier indices execute earlier only when the schedule keeps neighbors together in the same chunk. 
Otherwise, across threads, there is no bias — the probability is roughly equal for either order.
*/