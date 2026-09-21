from chain import contract as C
from chain.pattern import PatternStream, expected_sample, lfsr_next, seed, to_int16


def test_lfsr_matches_the_contract_formula():
    s = 1
    for _ in range(100):
        want = (s >> 1) ^ (0xB400 if s & 1 else 0)
        s = lfsr_next(s)
        assert s == want
        assert 0 <= s <= 0xFFFF


def test_lfsr_is_maximal_length_and_never_sticks():
    s, seen = 1, set()
    for _ in range(65535):
        assert s != 0, "an all-zero state would stick forever"
        seen.add(s)
        s = lfsr_next(s)
    assert len(seen) == 65535 and s == 1


def test_signed_conversion():
    assert to_int16(0x0000) == 0
    assert to_int16(0x7FFF) == 32767
    assert to_int16(0x8000) == -32768
    assert to_int16(0xFFFF) == -1


def test_check_mode_value_is_the_state_read_signed():
    st = PatternStream(realistic=False, n_ch=8)
    state = [seed(ch) for ch in range(8)]
    for _ in range(50):
        assert st.next_sample() == [to_int16(s) for s in state]
        state = [lfsr_next(s) for s in state]


def test_realistic_mode_matches_the_contract_formula():
    st = PatternStream(realistic=True, n_ch=8)
    state = [seed(ch) for ch in range(8)]
    for i in range(300):  # past one full 256-sample triangle
        got = st.next_sample()
        for ch in range(8):
            phase = (i + 8 * ch) % 256
            tri = phase * 128 - 8192 if phase < 128 else (255 - phase) * 128 - 8192
            assert got[ch] == tri + (state[ch] >> 12) - 8
            assert -32768 <= got[ch] <= 32767
        state = [lfsr_next(s) for s in state]


def test_random_access_matches_iterating_from_reset():
    for realistic in (False, True):
        st = PatternStream(realistic=realistic, n_ch=C.N_CH)
        ref = [st.next_sample() for _ in range(9000)]  # spans two memo checkpoints
        for i in (0, 1, 4095, 4096, 8999, 17, 5000, 3):  # deliberately out of order
            assert expected_sample(i, realistic) == ref[i]


def test_channels_differ_because_the_seed_differs():
    values = PatternStream(n_ch=C.N_CH).next_sample()
    assert len(values) == C.N_CH
    assert len(set(values)) == C.N_CH
