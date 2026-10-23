import sys,importlib
print('Python:',sys.version)
for name in ('numpy','scipy','matplotlib','cv2','sklearn','joblib'):
 try:
  m=importlib.import_module(name);print(name,getattr(m,'__version__','available'),'OK')
 except ImportError as e:raise RuntimeError(f'Missing {name}: pip install -r requirements.txt') from e
print('Webots simulation requires separate Webots installation; controller module exists only inside Webots.')
