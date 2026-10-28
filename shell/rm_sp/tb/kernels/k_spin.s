# k_spin -- a long-running kernel for the reset-during-dispatch test (abort via WDIST_CTRL).
# One warp (32 lanes).  A = u[10:11] (ro), OUT = u[14:15]: 6000 times
#   acc += A[lane]; OUT[lane] = acc
# i.e. a load and a store per iteration (requests on the fabric when the dispatch is aborted); a few ms of SP time.
        laneid  v1
        sll.vi  v2, v1, 2                        # lane * 4
        movi    v4, 0
        umovi   u40, 6000
loop:   ld.ubase.b32 v3, u[10:11], v2, 0, def
        waitcnt 0, 31, 31, 15, 3, 7
        add     v4, v4, v3
        st.ubase.b32 v4, u[14:15], v2, 0, def
        uaddi   u40, u40, -1
        bnz     u40, loop
        waitcnt 63, 0, 31, 15, 3, 7
        endpgm
