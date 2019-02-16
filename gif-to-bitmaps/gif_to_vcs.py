from PIL import Image
import sys
import os

''' NOTE: THIS DOESN'T SUPPORT TRANSPARANCY! '''

'''This function returns the amount of frames in a gif. Seeks back to 0 at end.'''
def get_frame_count(im):
    frames = 0
    try:
        while 1:
            im.seek(im.tell()+1)
            frames += 1
    except EOFError:
        pass # end of sequence

    im.seek(0)
    return frames;

'''This function reads a gif file, converts in into its individual pieces, then returns the cpp formatted info as a
    single string that is fit to be written to a file.'''
def process_image(infile):
    try:
        im = Image.open(infile)
    except IOError:
        print ("Cant load gif, sorry!", infile)
        sys.exit(1)
    i = 0
    mypalette = im.getpalette()

    output_string = '{{"{}","{}","{}"}}'.format(get_frame_count(im), im.size[0], im.size[1])
    print("frames: {}".format(get_frame_count(im)))
    try:
        # Loop through each frame and save the data.
        while True:
            # Get the current gif frame
            im.putpalette(mypalette)
            new_img = Image.new("RGBA", im.size)
            new_img.paste(im)

            # Save image metadata.
            pixel_count = im.size[0] * im.size[1]
            output_string += ',\n{{"{}","{}"'.format(im.info['duration'], pixel_count)

            # Format image data into hex colours, then compile into a c++ list.
            new_img_data = new_img.getdata()
            for color in new_img_data:
                output_string += ',"#{}"'.format( hex(color[0])[1:-1] + ('0' if color[0]<16 else '') +
                                                  hex(color[1])[1:-1] + ('0' if color[0]<16 else '') +
                                                  hex(color[2])[1:-1] + ('0' if color[0]<16 else '') )

            output_string += '}'

            i += 1
            im.seek(im.tell() + 1)
            print("image #{}".format(i))

    except EOFError: 
        pass # This exception is called just after the last image is proccessed.

    return output_string

# Process Gif
path = raw_input("Input the path of the gif file to convert:")
print("Processing... (This may take a few minutes because python is pretty slow and image manipulations take time)")
formatted_variable_info = process_image(path)
print(len(formatted_variable_info))

# Format Strings
header_comment = "// MUST INCLUDE:\n#include <string>\n#include <vector>\n\n"
comment = "// Rename this variable to whatever you want.\n"
variable_body = "std::vector< std::vector<std::string> > stringPointer = {{{}\n}};".format(formatted_variable_info)
print(len(variable_body))

# Write info to fil
f_number = 0
while ( os.path.isfile("{}\\gif_output{}.txt".format(os.getcwd(), f_number)) ):
   f_number += 1

print("Writing to file at {} ...".format(os.getcwd()))
file_out = open("{}/gif_output{}.txt".format(os.getcwd(), f_number), "w")
file_out.write(header_comment + comment + variable_body);

print("Done! Copy the code from the file 'gif_output{}.txt' and paste it into vcs. \n ^^".format(f_number))
