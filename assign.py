class T():
    def __assignpre__(self, lhs_name, rhs_name, rhs):
       print('%s has been copied to %s' % (rhs_name, lhs_name))
       return rhs

    def __assignpost__(self, lhs_name, rhs_name):
       print('POST: lhs', self)
       print('POST: lhs_name', lhs_name)
       print('POST: rhs_name', rhs_name)
       print('POST: assigning %s = %s' % (lhs_name, rhs_name))
       self.name = lhs_name

b = T()
# b = 10
# print(b)