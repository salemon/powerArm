from optimization import dynamic_optimization
import numpy as np
from scipy.optimize import minimize

opt = dynamic_optimization()
init_params = np.zeros(opt.num_links * 10 + 4)
init_params = opt.model2Params(init_params)

# res = minimize(opt.objectiveFunction, init_params, method='BFGS', callback = opt.callback, bounds=opt.bnds, options={'xrtol': 1e-6, 'disp': True, 'return_all': True})
# res = minimize(opt.objectiveFunction, init_params, method='L-BFGS-B', callback = opt.callback, bounds=opt.bnds, options={'disp': True})
res = minimize(opt.objectiveFunction, init_params, method='L-BFGS-B', callback = opt.callback, options={'disp': True, "gtol": 1e-6})

print(res.x)
with open('optimized_params_2.npy', 'wb') as f:
    np.save(f, res.x)