# k_atom -- uc (SYS) atomics through the M1 RM fabric bridge (ng_rsp_fab_l2 read-modify-write).
# One warp (32 lanes).  CNT = u[12:13], OUT = u[14:15]:
#   CNT[0] += lane (add.rtn, old -> OUT[lane])        CNT[1] = max(CNT[1], 7 * lane)
#   CNT[2] inc wrap 20 (inc.rtn, old -> OUT[32+lane])  CNT[3] ^= lane
#   CNT[4+lane] cmpxchg(0 -> lane + 100) (old -> OUT[64+lane]), CNT64 at +0x100 += lane
        laneid  v1
        sll.vi  v2, v1, 2                        # lane * 4
        movi    v10, 0                           # common counter offset
        mov     v3, v1
        atom.ubase.add.u32.rtn v3, u[12:13], v10, 0, def
        mul.vi  v4, v1, 7
        atom.ubase.max.u32 v4, u[12:13], v10, 4, def
        movi    v5, 20
        atom.ubase.inc.u32.rtn v5, u[12:13], v10, 8, def
        mov     v6, v1
        atom.ubase.xor.b32 v6, u[12:13], v10, 12, def
        movi    v8, 0
        movi    v7, 100
        add     v9, v1, v7
        atom.ubase.cmpxchg.b32.rtn v[8:9], u[12:13], v2, 16, def
        mov     v12, v1
        movi    v13, 0
        atom.ubase.add.u64 v[12:13], u[12:13], v10, 256, def
        waitcnt 0, 0, 31, 15, 3, 7
        st.ubase.b32 v3, u[14:15], v2, 0, def
        st.ubase.b32 v5, u[14:15], v2, 128, def
        st.ubase.b32 v8, u[14:15], v2, 256, def
        waitcnt 63, 0, 31, 15, 3, 7
        endpgm
