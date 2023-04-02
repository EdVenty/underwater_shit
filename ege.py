import itertools


def t1():
    c = 0
    for i in itertools.product("МЕТРО", repeat=4):
        if i[0] in "МТР" and i[-1] in "ЕО":
            c += 1

    print(c)

def t2():
    def to_base(number, base):
        if number == 0:
            yield 0
        while number > 0:
            number, digit = divmod(number, base)
            yield digit

    position = 138
    alphabet = "МСТФ"
    number = list(to_base(position - 1, len(alphabet)))
    number.reverse()
    word_without_leading_zeroes = "".join([alphabet[i] for i in number])
    word = alphabet[0] * (len(alphabet) - len(word_without_leading_zeroes)) + word_without_leading_zeroes
    print(word)

def t3():
    c = 0
    for i in itertools.product(range(1, 5), repeat=5):
        if i.count(1) == 2:
            c += 1
    print(c)

def t4():
    c = 0
    for i in itertools.product("ABCX", repeat=5):
        if i.count('X') == 0 or (i[-1] == 'X' and i.count('X') == 1):
            c += 1
    print(c)

def t5():
    def to_base(number, base):
        if number == 0:
            yield 0
        while number > 0:
            number, digit = divmod(number, base)
            yield digit

    position = 125
    alphabet = "АОУ"
    number = list(to_base(position - 1, len(alphabet)))
    number.reverse()
    word_without_leading_zeroes = "".join([alphabet[i] for i in number])
    word = alphabet[0] * (len(alphabet) - len(word_without_leading_zeroes)) + word_without_leading_zeroes
    print(word)

def t6():
    alphabet = "АГИЛМОРТ"
    start = ('И', 'Г')
    for indx, i in enumerate(itertools.product(alphabet, repeat=4)):
        if i[:2] == start:
            print(indx + 1)
            return

t6()