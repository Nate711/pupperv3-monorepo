import io
from PIL import Image as PILImage
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

def pil_to_compressed_msg(
    pil_img: PILImage,
    fmt: str = "jpeg",          # "jpeg", "png", "bmp", ...
    quality: int = 90,
    frame_id: str = "camera",
    stamp=None,
    **pil_save_kwargs,          # extra PIL save options if you want
) -> CompressedImage:
    fmt = fmt.lower()

    # JPEG can't have alpha; normalize to a sane mode for encoders
    if fmt in ("jpg", "jpeg"):
        if pil_img.mode not in ("RGB", "L"):
            pil_img = pil_img.convert("RGB")
    elif pil_img.mode == "P":     # paletted images can confuse some encoders
        pil_img = pil_img.convert("RGBA" if "A" in pil_img.getbands() else "RGB")

    buf = io.BytesIO()
    save_kwargs = dict(pil_save_kwargs)
    if fmt in ("jpg", "jpeg"):
        save_kwargs.setdefault("quality", quality)
        save_kwargs.setdefault("optimize", True)
        save_kwargs.setdefault("progressive", True)
    elif fmt == "png":
        # 0 (fastest) .. 9 (smallest). PIL default is 9; 6 is a common balance.
        save_kwargs.setdefault("compress_level", 6)
        save_kwargs.setdefault("optimize", True)

    pil_img.save(buf, format=fmt.upper(), **save_kwargs)
    data = buf.getvalue()

    msg = CompressedImage()
    msg.header = Header()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = fmt
    msg.data = data
    return msg
