from moviepy.editor import VideoFileClip


def dump(filename, output_dir, process_image):
    clip = VideoFileClip(filename)
    res = clip.fl_image(process_image)
    res.write_videofile(output_dir + filename, audio=False)
