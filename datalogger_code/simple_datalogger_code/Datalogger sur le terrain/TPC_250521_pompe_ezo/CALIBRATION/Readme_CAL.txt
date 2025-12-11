Pour calibrer la pompe utiliser le code arduino_UNO_PMP_sample_code.ino.

Mettre 9600 baud dans le serial.

Taper les commandes directement dans le la barre du serial par exemple

D,50 (injection de 50 ml)

Le plus pratique est de prendre un ballon jaugé de 100 ml. 
On envoie alors la commande D,100
Si la quantité injectée dans le ballon est inférieur à 100 ml, on lui indique la quantité qu'on estime être injectée et on envoie la commande
Cal,95

On envoie à nouveau la commande D,100 pour voir si la correction est bonne. S'il injecte plus de 100 ml, on peut envoyer Cal,103 et ainsi de suite

