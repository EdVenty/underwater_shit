import forbiddenfruit
import inspect
import linecache

# @forbiddenfruit.curses(float, '__assignpost__')
# def __assignpost__(self, *args):
#     print("post")

susumus = None

@forbiddenfruit.curses(object, '__assignpre__')
def __assignpre__(self, *args):
    global susmus
    susmus = args[2]
    # globals()[args[0]] = "sus"
    print(linecache.getline(__file__, inspect.stack()[1].lineno))
    return "sus"

a = 1.1
b = a
print(susumus)