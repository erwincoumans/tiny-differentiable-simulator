import pytinydiffsim_ad as pd

x = pd.ADDouble(3.)
x = pd.independent([x])
y = x[0]*x[0]*x[0]
f = pd.ADFun(x, [y])

print('derivative:', f.Reverse(1, [1.]))

