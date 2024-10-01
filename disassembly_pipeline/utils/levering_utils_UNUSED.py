import numpy as np
import rospy

#dz = -0.2
#EE_T_K = np.array([[1,0,0,0],
#                  [0,1,0,0],
#                  [0,0,1,dz],
#                  [0,0,0,1]])
#reshaped_EE_T_K = np.reshape(EE_T_K, newshape=16, order = 'f') # F for column major
#reshaped_EE_T_K
#p.SetKFrame(reshaped_EE_T_K)

#dz = +0.2
#NE_T_EE = np.array([[1,0,0,0],
#                  [0,1,0,0],
#                  [0,0,1,dz],
#                  [0,0,0,1]])
#reshaped_NE_T_EE = np.reshape(NE_T_EE, newshape=16, order = 'f') # F for column major
#reshaped_NE_T_EE
#p.SetEEFrame(reshaped_NE_T_EE)

def chkros():
        """ Check if ros is shutdown"""
        if rospy.is_shutdown(): return 0
        return 1

def dist2lines(p1,p12,p2,p22):
    '''DIST2LINES Shortest distance between two lines defined by points and direction
     Usage:
       [dist,points]=dist2lines(p1,n1,p2,n2)

     Input:
       p1  point on line 1 (3 x 1)
       p12 second point on line 2 (3 x 1)
       p2  point on line 2 (3 x 1)
       p22 second point on line 2 (3 x 1)

     Output:
       dist distance between lines
       pts  closest points on both lines (2 x 3) 

    '''
    p1=p1[:]
    d1=p1-p12
    p2=p2[:]
    d2=p2-p22

    d12=p2-p1

    n1=(np.matmul(d1,d1))    #%D1=sum(d1.^2);
    n2=(np.matmul(d2,d2))    #%D2=sum(d2.^2);

    S1=np.matmul(d1,d12)   #% S1=sum(d1.*d12);
    S2=np.matmul(d2,d12)   #% S2=sum(d2.*d12);
    R =np.matmul(d1,d2)    #% R=sum(d1.*d2);

    den=n1*n2-R**2    #denominator

    if (n1==0 or n2==0):       # if one of the segments is a point
        if (n1!=0):            # if line1 is a segment and line2 is a point
            u=0
            t=S1/n1
            t=t
        elif (n2!=0):        # if line2 is a segment and line 1 is a point
            t=0
            u=-S2/n2
            u=u
        else:                  # both segments are points
            t=0          
            u=0

    elif (den==0):        # if lines are parallel
        t=0
        u=-S2/n2
        uf=u
        if (uf!= u):
            t=(uf*R+S1)/n1
            t=t
            u=uf

    else:                        # general case
        t=(S1*n2-S2*R)/den;
        #t=t;
        u=(t*R-S2)/n2;
        uf=u;
        if (uf!=u):
            t=(uf*R+S1)/n1
            t=t
            u=uf

    # Compute distance given parameters 't' and 'u'
    dist=np.linalg.norm(d1*t-d2*u-d12)   # dist=sqrt(sum((d1*t-d2*u-d12).^2));

    points=np.array([p1+d1*t, p2+d2*u])

    return dist,points

def EE_to_fulcrum_dist(EE_1, EE_2, EE_T_K):
    """Input parameters as follows:
    -- EE_1 - first T matrix, while robot is touching fulcrum
    -- EE_2 - second T matrix while robot is touching fulcrum
    -- K_T_EE - relationship between K and EE frames (usually constant)

    returns:
    [x,y,z] of the fulcrum location
    """
    if (type(EE_T_K) == tuple) or (EE_T_K.shape == 16):
        EE_T_K = np.array(EE_T_K).reshape((4,4,), order = 'F')
    KK_1 = EE_1@EE_T_K
    KK_2 = EE_2@EE_T_K

    dist, points = dist2lines(EE_1[0:3,-1] , KK_1[0:3,-1], EE_2[0:3,-1], KK_2[0:3,-1])
    avg_point = (points[0,:] + points[1,:])/2

    #a = np.array([x,z])
    #b = np.array([EE_1[0,-1], EE_1[2,-1]])
    #c = a-b
    c = EE_2[0:3,-1] - avg_point
    z_dist = np.linalg.norm(c)

    return avg_point, z_dist

def line_intersection_UNUSED(line1, line2):
    """ Unused, only works on planar lines """
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
        raise Exception('lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y
