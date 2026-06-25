from manip_audio import speech_node


def test_speech_node_entry_point_is_available():
    assert callable(speech_node.main)


def test_normalize_text_removes_pause_artifacts():
    assert (
        speech_node.SpeechNode._normalize_text(
            '  tag_1\n encontrada  '
        ) == 'tag 1 encontrada'
    )


def test_speech_command_uses_smoother_espeak_settings():
    node = speech_node.SpeechNode.__new__(speech_node.SpeechNode)
    node._executable = 'espeak-ng'
    node._voice = 'pt-br'
    node._rate = 165
    node._pitch = 45
    node._word_gap = 0
    node._volume = 100

    command = node._speech_command('Teste de voz')

    assert command == [
        'espeak-ng',
        '-v', 'pt-br',
        '-s', '165',
        '-p', '45',
        '-g', '0',
        '-a', '100',
        '-z',
        'Teste de voz',
    ]
