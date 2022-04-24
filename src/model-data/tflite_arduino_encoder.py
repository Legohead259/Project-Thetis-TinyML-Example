# ==========================================================
# ===== ENCODE TENSORFLOW LITE MODEL AS ARDUINO HEADER =====
# ==========================================================

# from subprocess import call
# call("echo ""const char const unsigned char model[] = {"" > model.h")
# call("cat gesture_model.tflite | xxd -i >> model.h")
# call("echo ""};"" >> model.h")

# from hexdump import dump

# with open("model.h", 'w') as model_header:
#     model_header.write("const unsigned char model[] = {")

# with open("model.h", "a") as model_header, open("gesture_model.tflite", 'rb') as model_file:
#     model_header.write(dump(model_file.read()))
#     model_header.close()

# with open("model.h", 'a') as model_header:
#     model_header.write("};")

import os
model_h_size = os.path.getsize("model.h")
print(f"Header file, model.h, is {model_h_size:,} bytes.")