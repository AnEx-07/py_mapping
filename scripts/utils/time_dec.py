from time import time

def monitor(func):
    def func_wrapper(*a,**kw):
        t0 = time()
        ret = func(*a,**kw)
        print(func, "takes:",time()-t0,"s.")
        return ret
    return func_wrapper