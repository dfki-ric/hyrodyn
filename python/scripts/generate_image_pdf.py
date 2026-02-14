import matplotlib.pyplot as plt
import math 
import os
from matplotlib.backends.backend_pdf import PdfPages

def remove_borders_from_images(folder):
	path_to_trimmed_pics = os.path.join(folder, "trimmed")
	if not os.path.exists(path_to_trimmed_pics):
		os.mkdir(path_to_trimmed_pics)
	for filename in os.listdir(folder):
		os.system("convert " + os.path.join(folder, filename) + " -trim " + os.path.join(path_to_trimmed_pics, filename))

def load_images_from_folder(folder):
	"""Load all images from a folder.
	Parameters
	----------
	folder : string
	Path to the folder in which the images are stored
	Returns
	-------
	images : list of arrays
	List with images readed as arrays
	"""
	images = []
	for filename in os.listdir(folder):
		img = plt.imread(os.path.join(folder, filename))
		if img is not None:
			images.append(img)
	return images


# display all images in one figure
def display_multiple_img(images):
	"""Display all images on one site and save it as pdf file.
	Parameters
	----------
	images : list of arrays
	List with images readed as arrays
	Returns
	-------
	Saves the subplots as one page of a pdf
	"""
	nrows = math.ceil(len(images)/2)
	ncols = 2
	#with PdfPages("Workspace_Analysis") as pdf:
	fig = plt.figure(figsize=(10,10)) # Notice the equal aspect ratio
	ax = [plt.subplot(nrows,ncols,i+1) for i in range(len(images))]

	for a,img in zip(ax,images):
		a.imshow(img)
		a.set_title("bla")
		a.axis("off")

	#plt.subplots_adjust(wspace=0, hspace=0)
	#plt.show()
	fig.savefig('workspace_analysis.pdf', format='pdf', dpi=300)

# display all images in one figure
def glue_multiple_images(folder, ncols=2, generate_image_title = True):
	images = os.listdir(folder)
	print("Images in this folder:", images)
	nrows = math.ceil(len(images)/ncols)
	fig = plt.figure(figsize=(8.27, 11.69)) # A4 size paper
	ax = [plt.subplot(nrows,ncols,i+1) for i in range(len(images))]
	for a,img in zip(ax,images):
		i = plt.imread(os.path.join(folder, img))
		a.imshow(i)
		if generate_image_title is True:
			img_title = os.path.splitext(img)[0].replace("_", " ").capitalize()	# remove the file extension, replace underscore with space, capitalize
			a.set_title(img_title)
		a.axis("equal")
		a.axis("off")

	#plt.show()
	fig.savefig('workspace_analysis.pdf', format='pdf', dpi=300)

# Load all the images
#images = load_images_from_folder(path_to_trimmed_pics)
# Export multiple images in a single pdf
#display_multiple_img(images)
# go to the workspace folder

img_folder_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/python/results/lower_body/workspace"

# Remove borders from images
remove_borders_from_images(img_folder_path)
path_to_trimmed_pics = os.path.join(img_folder_path, "trimmed")
# Glue the images
glue_multiple_images(path_to_trimmed_pics)

