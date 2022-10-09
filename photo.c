/*									tab:8
 *
 * photo.c - photo display functions
 *
 * "Copyright (c) 2011 by Steven S. Lumetta."
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice and the following
 * two paragraphs appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE AUTHOR OR THE UNIVERSITY OF ILLINOIS BE LIABLE TO 
 * ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL 
 * DAMAGES ARISING OUT  OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, 
 * EVEN IF THE AUTHOR AND/OR THE UNIVERSITY OF ILLINOIS HAS BEEN ADVISED 
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE AUTHOR AND THE UNIVERSITY OF ILLINOIS SPECIFICALLY DISCLAIM ANY 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE 
 * PROVIDED HEREUNDER IS ON AN "AS IS" BASIS, AND NEITHER THE AUTHOR NOR
 * THE UNIVERSITY OF ILLINOIS HAS ANY OBLIGATION TO PROVIDE MAINTENANCE, 
 * SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Author:	    Steve Lumetta
 * Version:	    3
 * Creation Date:   Fri Sep  9 21:44:10 2011
 * Filename:	    photo.c
 * History:
 *	SL	1	Fri Sep  9 21:44:10 2011
 *		First written (based on mazegame code).
 *	SL	2	Sun Sep 11 14:57:59 2011
 *		Completed initial implementation of functions.
 *	SL	3	Wed Sep 14 21:49:44 2011
 *		Cleaned up code for distribution.
 */


#include <string.h>

#include "assert.h"
#include "modex.h"
#include "photo.h"
#include "photo_headers.h"
#include "world.h"


/* types local to this file (declared in types.h) */

/* 
 * A room photo.  Note that you must write the code that selects the
 * optimized palette colors and fills in the pixel data using them as 
 * well as the code that sets up the VGA to make use of these colors.
 * Pixel data are stored as one-byte values starting from the upper
 * left and traversing the top row before returning to the left of
 * the second row, and so forth.  No padding should be used.
 */
struct photo_t {
    photo_header_t hdr;			/* defines height and width */
    uint8_t        palette[192][3];     /* optimized palette colors */
    uint8_t*       img;                 /* pixel data               */
};

/* 
 * An object image.  The code for managing these images has been given
 * to you.  The data are simply loaded from a file, where they have 
 * been stored as 2:2:2-bit RGB values (one byte each), including 
 * transparent pixels (value OBJ_CLR_TRANSP).  As with the room photos, 
 * pixel data are stored as one-byte values starting from the upper 
 * left and traversing the top row before returning to the left of the 
 * second row, and so forth.  No padding is used.
 */
struct image_t {
    photo_header_t hdr;			/* defines height and width */
    uint8_t*       img;                 /* pixel data               */
};


/* file-scope variables */

/* 
 * The room currently shown on the screen.  This value is not known to 
 * the mode X code, but is needed when filling buffers in callbacks from 
 * that code (fill_horiz_buffer/fill_vert_buffer).  The value is set 
 * by calling prep_room.
 */
static const room_t* cur_room = NULL; 
void gen_palette(uint16_t * color_data, photo_t * p);

/* declare functions */

/* 
 * fill_horiz_buffer
 *   DESCRIPTION: Given the (x,y) map pixel coordinate of the leftmost 
 *                pixel of a line to be drawn on the screen, this routine 
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS: (x,y) -- leftmost pixel of line to be drawn 
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void
fill_horiz_buffer (int x, int y, unsigned char buf[SCROLL_X_DIM])
{
    int            idx;   /* loop index over pixels in the line          */ 
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgx;  /* loop index over pixels in object image      */ 
    int            yoff;  /* y offset into object image                  */ 
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo (cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_X_DIM; idx++) {
        buf[idx] = (0 <= x + idx && view->hdr.width > x + idx ?
		    view->img[view->hdr.width * y + x + idx] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate (cur_room); NULL != obj;
    	 obj = obj_next (obj)) {
	obj_x = obj_get_x (obj);
	obj_y = obj_get_y (obj);
	img = obj_image (obj);

        /* Is object outside of the line we're drawing? */
	if (y < obj_y || y >= obj_y + img->hdr.height ||
	    x + SCROLL_X_DIM <= obj_x || x >= obj_x + img->hdr.width) {
	    continue;
	}

	/* The y offset of drawing is fixed. */
	yoff = (y - obj_y) * img->hdr.width;

	/* 
	 * The x offsets depend on whether the object starts to the left
	 * or to the right of the starting point for the line being drawn.
	 */
	if (x <= obj_x) {
	    idx = obj_x - x;
	    imgx = 0;
	} else {
	    idx = 0;
	    imgx = x - obj_x;
	}

	/* Copy the object's pixel data. */
	for (; SCROLL_X_DIM > idx && img->hdr.width > imgx; idx++, imgx++) {
	    pixel = img->img[yoff + imgx];

	    /* Don't copy transparent pixels. */
	    if (OBJ_CLR_TRANSP != pixel) {
		buf[idx] = pixel;
	    }
	}
    }
}


/* 
 * fill_vert_buffer
 *   DESCRIPTION: Given the (x,y) map pixel coordinate of the top pixel of 
 *                a vertical line to be drawn on the screen, this routine 
 *                produces an image of the line.  Each pixel on the line
 *                is represented as a single byte in the image.
 *
 *                Note that this routine draws both the room photo and
 *                the objects in the room.
 *
 *   INPUTS: (x,y) -- top pixel of line to be drawn 
 *   OUTPUTS: buf -- buffer holding image data for the line
 *   RETURN VALUE: none
 *   SIDE EFFECTS: none
 */
void
fill_vert_buffer (int x, int y, unsigned char buf[SCROLL_Y_DIM])
{
    int            idx;   /* loop index over pixels in the line          */ 
    object_t*      obj;   /* loop index over objects in the current room */
    int            imgy;  /* loop index over pixels in object image      */ 
    int            xoff;  /* x offset into object image                  */ 
    uint8_t        pixel; /* pixel from object image                     */
    const photo_t* view;  /* room photo                                  */
    int32_t        obj_x; /* object x position                           */
    int32_t        obj_y; /* object y position                           */
    const image_t* img;   /* object image                                */

    /* Get pointer to current photo of current room. */
    view = room_photo (cur_room);

    /* Loop over pixels in line. */
    for (idx = 0; idx < SCROLL_Y_DIM; idx++) {
        buf[idx] = (0 <= y + idx && view->hdr.height > y + idx ?
		    view->img[view->hdr.width * (y + idx) + x] : 0);
    }

    /* Loop over objects in the current room. */
    for (obj = room_contents_iterate (cur_room); NULL != obj;
    	 obj = obj_next (obj)) {
	obj_x = obj_get_x (obj);
	obj_y = obj_get_y (obj);
	img = obj_image (obj);

        /* Is object outside of the line we're drawing? */
	if (x < obj_x || x >= obj_x + img->hdr.width ||
	    y + SCROLL_Y_DIM <= obj_y || y >= obj_y + img->hdr.height) {
	    continue;
	}

	/* The x offset of drawing is fixed. */
	xoff = x - obj_x;

	/* 
	 * The y offsets depend on whether the object starts below or 
	 * above the starting point for the line being drawn.
	 */
	if (y <= obj_y) {
	    idx = obj_y - y;
	    imgy = 0;
	} else {
	    idx = 0;
	    imgy = y - obj_y;
	}

	/* Copy the object's pixel data. */
	for (; SCROLL_Y_DIM > idx && img->hdr.height > imgy; idx++, imgy++) {
	    pixel = img->img[xoff + img->hdr.width * imgy];

	    /* Don't copy transparent pixels. */
	    if (OBJ_CLR_TRANSP != pixel) {
		buf[idx] = pixel;
	    }
	}
    }
}


/* 
 * image_height
 *   DESCRIPTION: Get height of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t 
image_height (const image_t* im)
{
    return im->hdr.height;
}


/* 
 * image_width
 *   DESCRIPTION: Get width of object image in pixels.
 *   INPUTS: im -- object image pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of object image im in pixels
 *   SIDE EFFECTS: none
 */
uint32_t 
image_width (const image_t* im)
{
    return im->hdr.width;
}

/* 
 * photo_height
 *   DESCRIPTION: Get height of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: height of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t 
photo_height (const photo_t* p)
{
    return p->hdr.height;
}


/* 
 * photo_width
 *   DESCRIPTION: Get width of room photo in pixels.
 *   INPUTS: p -- room photo pointer
 *   OUTPUTS: none
 *   RETURN VALUE: width of room photo p in pixels
 *   SIDE EFFECTS: none
 */
uint32_t 
photo_width (const photo_t* p)
{
    return p->hdr.width;
}


/* 
 * prep_room
 *   DESCRIPTION: Prepare a new room for display.  You might want to set
 *                up the VGA palette registers according to the color
 *                palette that you chose for this room.
 *   INPUTS: r -- pointer to the new room
 *   OUTPUTS: none
 *   RETURN VALUE: none
 *   SIDE EFFECTS: changes recorded cur_room for this file
 */
void
prep_room (const room_t* r)
{
    /* Record the current room. */
    cur_room = r;
	/*Get a pointer to the current room and use it to set the pallette*/
    photo_t * photo_struct = room_photo(r);
    /*Write the palette data to video memory*/
    set_palette((unsigned char *)photo_struct->palette);
}


/* 
 * read_obj_image
 *   DESCRIPTION: Read size and pixel data in 2:2:2 RGB format from a
 *                photo file and create an image structure from it.
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the image
 */
image_t*
read_obj_image (const char* fname)
{
    FILE*    in;		/* input file               */
    image_t* img = NULL;	/* image structure          */
    uint16_t x;			/* index over image columns */
    uint16_t y;			/* index over image rows    */
    uint8_t  pixel;		/* one pixel from the file  */

    /* 
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the image pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen (fname, "r+b")) ||
	NULL == (img = malloc (sizeof (*img))) ||
	NULL != (img->img = NULL) || /* false clause for initialization */
	1 != fread (&img->hdr, sizeof (img->hdr), 1, in) ||
	MAX_OBJECT_WIDTH < img->hdr.width ||
	MAX_OBJECT_HEIGHT < img->hdr.height ||
	NULL == (img->img = malloc 
		 (img->hdr.width * img->hdr.height * sizeof (img->img[0])))) {
	if (NULL != img) {
	    if (NULL != img->img) {
	        free (img->img);
	    }
	    free (img);
	}
	if (NULL != in) {
	    (void)fclose (in);
	}
	return NULL;
    }

    /* 
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order (top to bottom).
     */
    for (y = img->hdr.height; y-- > 0; ) {

	/* Loop over columns from left to right. */
	for (x = 0; img->hdr.width > x; x++) {

	    /* 
	     * Try to read one 8-bit pixel.  On failure, clean up and 
	     * return NULL.
	     */
	    if (1 != fread (&pixel, sizeof (pixel), 1, in)) {
		free (img->img);
		free (img);
	        (void)fclose (in);
		return NULL;
	    }

	    /* Store the pixel in the image data. */
	    img->img[img->hdr.width * y + x] = pixel;
	}
    }

    /* All done.  Return success. */
    (void)fclose (in);
    return img;
}


/* 
 * read_photo
 *   DESCRIPTION: Read size and pixel data in 5:6:5 RGB format from a
 *                photo file and create a photo structure from it.
 *                Code provided simply maps to 2:2:2 RGB.  You must
 *                replace this code with palette color selection, and
 *                must map the image pixels into the palette colors that
 *                you have defined.
 *   INPUTS: fname -- file name for input
 *   OUTPUTS: none
 *   RETURN VALUE: pointer to newly allocated photo on success, or NULL
 *                 on failure
 *   SIDE EFFECTS: dynamically allocates memory for the photo
 */
photo_t*
read_photo (const char* fname)
{
    FILE*    in;	/* input file               */
    photo_t* p = NULL;	/* photo structure          */
    uint16_t x;		/* index over image columns */
    uint16_t y;		/* index over image rows    */
    uint16_t pixel;	/* one pixel from the file  */

    /* 
     * Open the file, allocate the structure, read the header, do some
     * sanity checks on it, and allocate space to hold the photo pixels.
     * If anything fails, clean up as necessary and return NULL.
     */
    if (NULL == (in = fopen (fname, "r+b")) ||
	NULL == (p = malloc (sizeof (*p))) ||
	NULL != (p->img = NULL) || /* false clause for initialization */
	1 != fread (&p->hdr, sizeof (p->hdr), 1, in) ||
	MAX_PHOTO_WIDTH < p->hdr.width ||
	MAX_PHOTO_HEIGHT < p->hdr.height ||
	NULL == (p->img = malloc 
		 (p->hdr.width * p->hdr.height * sizeof (p->img[0])))) {
	if (NULL != p) {
	    if (NULL != p->img) {
	        free (p->img);
	    }
	    free (p);
	}
	if (NULL != in) {
	    (void)fclose (in);
	}
	return NULL;
    }

	uint16_t * raw_color_data = malloc(sizeof(pixel) * p->hdr.height * p->hdr.width);
    /* 
     * Loop over rows from bottom to top.  Note that the file is stored
     * in this order, whereas in memory we store the data in the reverse
     * order (top to bottom).
     */
    for (y = p->hdr.height; y-- > 0; ) {

	/* Loop over columns from left to right. */
	for (x = 0; p->hdr.width > x; x++) {

	    /* 
	     * Try to read one 16-bit pixel.  On failure, clean up and 
	     * return NULL.
	     */
	    if (1 != fread (&pixel, sizeof (pixel), 1, in)) {
		free (p->img);
		free (p);
	        (void)fclose (in);
		return NULL;
	    }

		 /*save the raw color data*/
        raw_color_data[p->hdr.width * y + x] = pixel;
	    /* 
	     * 16-bit pixel is coded as 5:6:5 RGB (5 bits red, 6 bits green,
	     * and 6 bits blue).  We change to 2:2:2, which we've set for the
	     * game objects.  You need to use the other 192 palette colors
	     * to specialize the appearance of each photo.
	     *
	     * In this code, you need to calculate the p->palette values,
	     * which encode 6-bit RGB as arrays of three uint8_t's.  When
	     * the game puts up a photo, you should then change the palette 
	     * to match the colors needed for that photo.
	     */
	    p->img[p->hdr.width * y + x] = (((pixel >> 14) << 4) |
					    (((pixel >> 9) & 0x3) << 2) |
					    ((pixel >> 3) & 0x3));
	}
    }
	/*Generate the color pallette and free the raw color data*/
    gen_palette(raw_color_data, p);
    /*Free the raw color data array because we no longer need it*/
    free(raw_color_data);

    /* All done.  Return success. */
    (void)fclose (in);
    return p;
}



/* constant to be used for generating an palette */


#define LEVEL2_SIZE 64
#define LEVEL4_SIZE 4096
#define BIT_MASK_RED 0xF800
#define BIT_MASK_GREEN 0X07E0
#define BIT_MASK_BLUE 0x001F

typedef struct node_t node_t;

/*This struct stands for the nodes of octrees,
 which contains index, count, RGB value.*/
struct node_t {
	int index;
	int count;
	long R_value;
	long G_value;
	long B_value;
};

/* 
 * 	 compare
 *   DESCRIPTION: receive two values of  node_t type and define a comparation rule of it
 *   INPUTS: const void* A, const void* B
 *   OUTPUTS: whether A is bigger then B
 *   RETURN VALUE: 1 or 0
 *   SIDE EFFECTS: none
 */
int compare(const void* A, const void* B)
{
	if ((*(node_t*)A).count < (*(node_t*)B).count)
		return 1;
	else
		return 0;
}

/* 
 * 	 compare
 *   DESCRIPTION: get a data of 16 bit and find its index in level4 of octree
 *   INPUTS: 16bit color data of RRRRRGGGGGGBBBBB
 *   OUTPUTS: index in level4 of octree
 *   SIDE EFFECTS: none
 */
uint16_t get_level4_index(uint16_t data)
{
	unsigned short R,G,B;
	uint16_t return_value;
	R=((data) & 0xF000) >> 12;
	G=((data) & 0x0780) >> 7;
	B=((data) & 0x001E) >> 1;
	return_value = (R << 8) | (G << 4) | (B << 0);
	return return_value;
}

/* 
 * 	 compare
 *   DESCRIPTION: get a data of 16 bit and find its index in level2 of octree
 *   INPUTS: 16bit color data of RRRRRGGGGGGBBBBB
 *   OUTPUTS: index in level4 of octree
 *   SIDE EFFECTS: NONE
 */
uint16_t get_level2_index(uint16_t data)
{
	unsigned short R,G,B;
	uint16_t return_value ;
	R=((data) & 0xC000) >> 14;
	G=((data) & 0x0600) >> 9;
	B=((data) & 0x0018) >> 3;
	return_value = (R << 4) | (G << 2) | (B << 0);
	return return_value;
}


/* 
 * 	 compare
 *   DESCRIPTION: Given a pointer to a photo data and a pointer to the struct of photo_t, generate a palette of 192*3 in it and 
 *   INPUTS: 
 *   OUTPUTS: 
 *   RETURN VALUE: 
 *   SIDE EFFECTS: 
 */
void gen_palette(uint16_t * color_data, photo_t * p)
{
      node_t level2[LEVEL2_SIZE];                         
      node_t level4[LEVEL4_SIZE];     
	  int index4_array[LEVEL4_SIZE];     /* index4 array stands for the dirction in sorted level4*/       

      int level4index, level2index;               
      int i;         
      long R, G, B;    

	  /* initialize the octree nodes */
      for(i = 0; i < LEVEL2_SIZE; i++){
            level2[i].index = i;
			level2[i].count = 0;
			level2[i].R_value = 0 ;
			level2[i].G_value = 0 ;
			level2[i].B_value = 0 ;
      }
      for(i = 0; i < LEVEL4_SIZE; i++){
            level4[i].index = i;
			level4[i].count = 0;
			level4[i].R_value = 0 ;
			level4[i].G_value = 0 ;
			level4[i].B_value = 0 ;
      }

	  /* First we load the level4 of octreee*/
      for(i = 0; i < (p->hdr.height * p->hdr.width); i++)
	  {
    
            level4index = get_level4_index(color_data[i]);

            R = ((color_data[i] & BIT_MASK_RED) >> 11) << 1;    /*11 stands for red offset*/
            G = ((color_data[i] & BIT_MASK_GREEN) >> 5) << 0;	/* 5 stands for green offset*/
            B = ((color_data[i] & BIT_MASK_BLUE) >> 0) << 1;	/* 0 stands for blue offset*/

			/* load the sum of RGB into R,G,B value of octree node,and count ++*/

            level4[level4index].R_value += R ;
            level4[level4index].G_value += G ;
            level4[level4index].B_value += B ;
			level4[level4index].count += 1;
			/*
			level2[level2index].R_value += R ;
			level2[level2index].G_value += G ;
			level2[level2index].B_value+= B ;
			level2[level2index].count += 1;
			*/

      }

	/*
	for (i=0;i<LEVEL2_SIZE;i++)
	{
		if (level2[i].count==0)
			continue;
		level2[i].R_value = level2[i].R_value / level2[i].count ;
		level2[i].G_value = level2[i].G_value / level2[i].count ;
		level2[i].B_value = level2[i].B_value / level2[i].count ;
	}*/
	
	  /*qsort the level4*/
      qsort(level4, LEVEL4_SIZE, sizeof(node_t), compare);

	  /*load the first 128 RGB of sorted level 4 and caculate the average*/
	  /* 128 stands for 128 colors*/
	  for (i=0;i<128;i++)
	  {
			if (level4[i].count==0)
				continue;
			level4[i].R_value = level4[i].R_value / level4[i].count;
			level4[i].G_value = level4[i].G_value / level4[i].count;
			level4[i].B_value = level4[i].B_value / level4[i].count;
	  }
	  /*load the index4_array*/
	  /*4096 stands for level4 size*/
	  for (i=0;i<4096;i++)
	  {
			index4_array[level4[i].index]=i;
	  }
	  /*load the sum of RGB value for the left values*/
	  for (i=128;i<4096;i++)
	  {
			level2index=(((level4[i].index & 0xC00)>>10)<<4)+(((level4[i].index & 0x0C0) >> 6) << 2)+((level4[i].index & 0x00C) >>2);
			level2[level2index].R_value += level4[i].R_value ;
			level2[level2index].G_value += level4[i].G_value ;
			level2[level2index].B_value += level4[i].B_value ;
			level2[level2index].count   += level4[i].count   ;
	  }
      /*caculate the average for the left ajfl*/
		for (i=0;i<LEVEL2_SIZE;i++)
		{
			if (level2[i].count==0)
				continue;
			level2[i].R_value = level2[i].R_value / level2[i].count ;
			level2[i].G_value = level2[i].G_value / level2[i].count ;
			level2[i].B_value = level2[i].B_value / level2[i].count ;
		}

      for(i = 0; i < (p->hdr.height * p->hdr.width); i++)
	  {
			level4index=index4_array[get_level4_index(color_data[i])];
			level2index=get_level2_index(color_data[i]);
			if (level4index < 128)
				p->img[i] = 64 + level4index ;
			else	
            	p->img[i] = 64 + 128 + level2index ;
      }

      for(i = 0; i < 128; i++){
            p->palette[i][0] = level4[i].R_value;
            p->palette[i][1] = level4[i].G_value;
            p->palette[i][2] = level4[i].B_value;
      }
      for(i = 0; i < 64; i++){
            p->palette[i+128][0] = level2[i].R_value;
            p->palette[i+128][1] = level2[i].G_value;
            p->palette[i+128][2] = level2[i].B_value;
      }
      return;
}