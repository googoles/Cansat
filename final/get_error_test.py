class Test:
    def study(self):
        raise OSError

yun = Test()
yun.study()