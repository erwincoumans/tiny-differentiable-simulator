import pytinydiffsim_dual as pd

def auto_diff(f, x):
    return f(pd.TinyDualDouble(x, 1.)).dual()


x = pd.TinyDualDouble(2,1)
y = pd.TinyDualDouble(3,0)

print(x)
print(y)
f = (x*x)*x
print(f)

print("pytinydiffsim.TinyDualDouble")
print(auto_diff(lambda x:pd.TinyDualDouble(1.,0)/(x*x*x*x*x), 0.01))
  