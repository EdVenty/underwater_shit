from encodings import utf_8
import encodings
import codecs
import io
import re

static_regex = r'^(\s*)([^\s]+) = static\(.*\)$'
assign_regex = r'^(\s*)([^\s]+) = (.*)$'
define_func_regex = r'def\s+(\S+)\s*\(.*\)\s*:'
operator_comment_regex = r'#\s*operator\s+(\S+)\s+(\S+)'
operator_decorator_regex = r"\s*@operator\(\"(.*)\"\)"
override_regex = r"([\"\']?\w+[\"\']?)\s*({})\s*([\"\']?\w+[\"\']?)"

def cpreprocessor_decode(input, errors='strict'):
    stream = io.StringIO(bytes(input).decode('utf-8'))
    output = ''.join(stream.readlines())
    output = preprocess(output)
    return output, len(input)

def preprocess_operators(code: str):
    operators_line = None
    operators = set()
    lines = code.split('\n')
    for i, line in enumerate(lines.copy()):
        if line == "# *place operators here*":
            # print(f"Operators line: {i} ({i + 1})")
            operators_line = i
        match_operator = re.match(operator_decorator_regex, line)
        if match_operator:
            name = match_operator.group(1)
            # print(f"Add operator: {name}")
            operators.add(name)
        difference = 0
        for operator in operators:
            pattern = override_regex.format("\\" + "\\".join(operator))
            for m in re.finditer(pattern, line):
                # print(i, m)
                replace = f"run_operator({m.group(1)}, {m.group(3)}, \"{m.group(2)}\")"
                # line[m.start() + difference : m.end() - 1 + difference] = replace
                lines[i] = re.sub(pattern, replace, line)
    lines[operators_line] = f"_operators = {dict.fromkeys(operators)}"
    print(lines[operators_line])
    return "\n".join(lines)
    

def preprocess(code):
    # static_variables = set()
    # splited = code.split('\n')
    # for i, line in enumerate(splited.copy()):
    #     match = re.match(static_regex, line)
    #     if match:
    #         static_variables.add(match.group(2))
    #         splited[i] = f"{match.group(1)}print('New static variable {match.group(2)}')"
    #         continue
    #     match_assign = re.match(assign_regex, line)
    #     if match_assign:
    #         var = match_assign.group(2)
    #         if var in static_variables:
    #             splited[i] = f"{match_assign.group(1)}_static_variables"
    #             # splited[i] = f"{match_assign.group(1)}print('Set static {match_assign.group(2)} to {match_assign.group(3)}')"
    #             continue
    #     match_def = re.match(define_func_regex, line)
    #     if match_def:
    #         ...
    # code = code.replace("++", "+=1")
    # for line in code.split('\n'):
    #     line = line.strip()
    #     if line.startswith('# define'):
    #         key, val = line[9:].split()
    #         code = code.replace(key, val)
    # return code
    # return "\n".join(splited)
    return preprocess_operators(code)
def search_function(encoding):
    # print("Preprocess")
    if encoding == 'cpreprocessor':
        utf8 = encodings.search_function('utf8')
        return codecs.CodecInfo(
            name='cpreprocessor',
            encode=utf8.encode,
            decode=cpreprocessor_decode,
            incrementalencoder=utf8.incrementalencoder,
            incrementaldecoder=utf_8.IncrementalDecoder,
            streamreader=utf_8.StreamReader,
            streamwriter=utf8.streamwriter)
codecs.register(search_function)
...