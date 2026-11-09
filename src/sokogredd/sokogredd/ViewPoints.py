import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from visualization_msgs.msg import Marker

import cv2 
import numpy as np 

LEDIG = 0
VEGG = 1
SETT = 2
UKJENT = -1
SCAN_FREQUANZY = 0.5 #1 da scannes hver rute, den må være < 1

def sirkel(radius_celler): #insanly kul måte å få en sirkelformet maske for scan radiusen vi trenger
    k = 2*int(np.ceil(radius_celler)) + 1 #finner maske størrelsen 
    return cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k,k)) #git ut en kxk numpy matrise med denne masken i midetn

#utvider masken for "naboene". dilate trenger tall så må omgjøre på veien
def utvid_for_sirkel(masken, mal):
    return cv2.dilate(masken.astype(np.int), mal).astype(bool)

#slår sammen "celle_størrelse x celle_størrelse" celler for å skalere ned. Dette da på w1 (bare den som er sjekket) er 2000x2000 som er alt for mye og bare lager mer styr
def ned_skaler_grid(grid, celle_størrelse):
    #Ai nevnte dette var feil og det skal være slikt i stedet:
    #h = grid.shape[0] // celle_størrelse
    #b = grid.shape[0] // celle_størrelse

    h = grid.shape[0] // celle_størrelse**2
    b = grid.shape[1] //celle_størrelse**2
    
    # grid.reshape(h, celle_størrelse, b, celle_størrelse)
    cellen = grid[:h, :b].reshape((h // celle_størrelse), celle_størrelse, (b // celle_størrelse), celle_størrelse) #veldig støgt ik men er bare cellen (i,j) i den nye ruten (R,C)
    Ikkr_vegger = np.where((cellen == UKJENT).any(axis=(1,3)), UKJENT, LEDIG)
    return np.where((cellen == VEGG).any(axis =(1,3)), VEGG, Ikkr_vegger).astype(np.int)

#cellene hver av raysene passerer, 2 rader, på form (antall stråler, antall steg), rad i = stråle i, kolonne j = steg j, 
def ray_cellene(grid, rad, kolonne, radius_celler, antall_rays):
    vinkler = np.linspace(0, 2 * math.pi, antall_rays, endpoint=False)
    distanser = np.arange(SCAN_FREQUANZY, radius_celler,SCAN_FREQUANZY)

    x_steg = kolonne + 0.5 + np.outer(math.cos(vinkler), distanser)
    y_steg = rad + 0.5 + np.outer(math.sin(vinkler), distanser)

    høyde, bredde = grid.shape

    #clip for å holde verdiene inne i kartet
    rader = np.clip(y_steg.astype(np.int), 0, høyde -1)
    kolonner = np.clip(x_steg.astype(np.int), 0, bredde - 1)

    return rader, kolonner

#hvor hver syråle som treffer noe treffer det for første gang- 
def forste_treff(grid, rader, kolonner):
    blokkert_celler = grid[rader, kolonner] != LEDIG
    ray_treffene = np.nonzero(np.any(blokkert_celler, axis=1))[0] #strålene som treffer noe annet enn 0 
    forste_treff = np.argmax(blokkert_celler[ray_treffene], axis=1) #argmax vil her gi index til første tilfelle for True for en ray  
    return rader[ray_treffene,forste_treff], kolonner[ray_treffene, forste_treff]

#markerer alle veggflatene som er synlig fra punktet 
def scan_rundt(grid, flate, rad, kolonne, radius_celler, antall_rays=700):
    rader = ray_cellene(grid, rad, kolonne, radius_celler, antall_rays)
    kolonner = ray_cellene(grid, rad, kolonne, radius_celler, antall_rays)
    rad_treff = forste_treff(grid, rader, kolonner)
    kolonne_treff = forste_treff(grid, rader, kolonner)
    return rad_treff[flate[rad_treff, kolonne_treff]], kolonne_treff[flate[rad_treff, kolonne_treff]]


#TODO
'''
1 en fyll funksjon som bruker flood fill til å sitte/finne fra punktet
2 en funksjon for å finne cellen i masken som ligger nærmest punktet den får inn, (rad, col)
3 en funksjon som tar å plaserer punkter der det er flest usette flater, som gjentar til alt er sett. 
4 ofc må vi da ha en finn flater som du trenger i 3), veggceller som står fritt område (1 ved siden av 0)
5 masse biter trenger en funksjon for å sitte det sammen, tada vi trenger en planlegg punkta funksjon som sitter det sammen og klart til node kjør under:))))))))
6 så, robotten skal starte et punkt, det vil være et åpent punkt, så da kan vi liksågodt ta en spin node der, og fylle:)..
7 åpen plass her om vi finner noe vi trenger 
8 og tada, til slutt kan vi bare smekke alt sammen i en kjør node, som bruker alt dette:) enkelt, super simpelt og raskt, nice, enkelt og gøy:) 
'''