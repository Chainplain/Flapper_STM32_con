Import("env", "projenv")

env.Append(
    LINKFLAGS=[
        "-mfloat-abi=hard",
        "-mfpu=fpv4-sp-d16"
    ],
)

