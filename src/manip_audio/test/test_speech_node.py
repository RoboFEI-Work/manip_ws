from manip_audio import speech_node


def test_speech_node_entry_point_is_available():
    assert callable(speech_node.main)
