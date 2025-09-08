import importlib.util
spec = importlib.util.spec_from_file_location('wui', r'c:\Users\Theminemat\Documents\Programming\explore_science\no_ai\web_ui\server.py')
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
print('Imported server module OK')
print('Front mock:', mod.sensor_front.get_distance_cm())
print('Right mock:', mod.sensor_right.get_distance_cm())
print('Heading sample:', mod.tracker.get_heading())
