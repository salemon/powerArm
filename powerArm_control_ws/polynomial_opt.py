import numpy as np
from scipy import odr
from optimization import dynamic_optimization

opt = dynamic_optimization()
poly_model = odr.polynomial(3)

n = 1
data = odr.Data(opt.q[:, n], opt.expTau[:, n])
odr_obj = odr.ODR(data, poly_model)

output = odr_obj.run()
poly = np.poly1d(output.beta[::-1])
poly_y = poly(opt.q[:, n])

print(np.sum((poly_y - opt.expTau[:, n])**2) / opt.num_data)
