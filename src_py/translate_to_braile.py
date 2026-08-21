# https://pypi.org/project/KorToBraille/#description
# https://braillify.kr/


import sys
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
            # 점자 이외의 문자는 그대로 표시
            result.append(char)

    return result


if __name__=="__main__" :

    text_k = "나는 로키"
    b = KorToBraille()
    text_b = b.korTranslate(text_k)
    print(text_b)
    print(type(text_b))
    print(len(text_b))

    # 변환
    bit_b = braille_text_to_bits(text_b)

    # 출력
    print(len(bit_b))
    print(bit_b)

