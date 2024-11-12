
                # read original waypoint file (lat,lon,alt)
def txt_reform(file_lacation):

            # set location folder for read
    f = open(f'wp/{file_lacation}', "r")
    
    data=f.readlines()
    n=0

    #heading
    txt = "QGC WPL 110 \n"

    # line 0 = set home (don't care)
    txt = txt + f"{n}\t0\t0\t16\t0\t0\t0\t0\t13.9185150\t100.6284700\t0.09\t1\n"
    for i in data:
        x = i.split() 
        n+=1
        print(x[0])
        print(x[1])
        print(x[2])

        txt = txt +f"{n}\t0\t3\t16\t0\t0\t0\t0\t{x[0]}\t{x[1]}\t{x[2]}\t1\n"
 
            # set location folder for write
    f = open(f're_wp/re_{file_lacation}','w')
    f.write(txt)
    f.close


                     
        # select file
txt_reform('simWP.txt')

    






    


