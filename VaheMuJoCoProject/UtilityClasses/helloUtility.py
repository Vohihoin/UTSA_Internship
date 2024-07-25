import cv2

def resizeImage(image, factor):
    
    h, w, c = image.shape
    return(cv2.resize(image, (int(w*factor), int(h*factor))))


# Very useful method. So, instead of worrying about creating individual lights to illuminate the scene, 
# I can create a string representing a ring of lights around the origin at a particular radius
# Then I can format that string into the xml text
# Starter is basically what number of lights to start county from (as in light 1, light2, etc)
# Radius is the radius of the ring
# height is how high the lights should originate from

def ringOfLightsXMLString(starter, radius, height):
    totalString = ""
    lightcounter = starter

    # center light
    lightLine = """ <light name = "overhead{lightnum}" pos = "0 0 {height}" dir = "0 0 -1" active = "true" />\n""".format(lightnum = lightcounter, height = height)
    totalString+= lightLine


    for i in range(radius):

        i = i + 1 #(0, 1, 2, ...) -> (1, 2, 3, ...)
        
        lightcounter += 1
        lightLine = """ <light name = "overhead{lightnum}" pos = "0 {r} {height}" dir = "0 0 -1" active = "true" />\n""".format(r = i, lightnum = lightcounter, height = height)
        totalString+= lightLine

        lightcounter += 1
        lightLine = """ <light name = "overhead{lightnum}" pos = "{r} 0 {height}" dir = "0 0 -1" active = "true" />\n""".format(r = i, lightnum = lightcounter, height = height)
        totalString+= lightLine

        lightcounter += 1
        lightLine = """ <light name = "overhead{lightnum}" pos = "0 -{r} {height}" dir = "0 0 -1" active = "true" />\n""".format(r = i, lightnum = lightcounter, height = height)
        totalString+= lightLine

        lightcounter += 1
        lightLine = """ <light name = "overhead{lightnum}" pos = "-{r} 0 {height}" dir = "0 0 -1" active = "true" />\n""".format(r = i, lightnum = lightcounter, height = height)
        totalString+= lightLine

    # produces 4r + 1 lights for the xml string
    return(totalString)



