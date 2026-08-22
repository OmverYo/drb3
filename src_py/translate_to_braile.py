from KorToBraille.KorToBraille import KorToBraille


def braille_text_to_bits(text: str) -> list:
    result = []

    for char in text:
        code = ord(char)

        # 점자 유니코드 범위: U+2800 ~ U+283F
        if 0x2800 <= code <= 0x283F:
            value = code - 0x2800

            bits = [
                (value >> 0) & 1,  # 점 1
                (value >> 1) & 1,  # 점 2
                (value >> 2) & 1,  # 점 3
                (value >> 3) & 1,  # 점 4
                (value >> 4) & 1,  # 점 5
                (value >> 5) & 1   # 점 6
            ]

            result.append(bits)

        else:
            result.append(char)

    return result


def reverse_braille(text: str) -> str:
    """점자 문자열을 좌우반전"""

    result = []

    for char in text:
        code = ord(char)

        if 0x2800 <= code <= 0x283F:
            value = code - 0x2800

            p1 = (value >> 0) & 1
            p2 = (value >> 1) & 1
            p3 = (value >> 2) & 1
            p4 = (value >> 3) & 1
            p5 = (value >> 4) & 1
            p6 = (value >> 5) & 1

            # 점자 좌우반전
            reversed_value = (
                (p4 << 0) |
                (p5 << 1) |
                (p6 << 2) |
                (p1 << 3) |
                (p2 << 4) |
                (p3 << 5)
            )

            result.append(chr(0x2800 + reversed_value))

        else:
            result.append(char)

    # 문자열의 글자 순서도 반전
    return ''.join(result[::-1])


if __name__ == "__main__":

    text_k = "나는 로키"

    b = KorToBraille()
    text_b = b.korTranslate(text_k)

    print("원본       :", text_b)

    text_b_reverse = reverse_braille(text_b)

    print("좌우반전    :", text_b_reverse)

    bit_b = braille_text_to_bits(text_b_reverse)

    #원래는 맨 끝 eol 문자였으나, 반전으로 앞으로 왔으니 날려줌.
    if bit_b[0] == [0, 0, 0, 0, 0, 0]:
        bit_b.pop(0)

    print("bit 길이   :", len(bit_b))
    print("bit        :", bit_b)