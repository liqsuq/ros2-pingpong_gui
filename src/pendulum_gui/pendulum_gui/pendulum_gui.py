import sys, os, time, subprocess, argparse
from logging import getLogger, StreamHandler, Formatter, DEBUG, INFO
from PySide2 import QtCore, QtGui, QtWidgets

import matplotlib
# Make sure that we are using QT5
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
import matplotlib.pyplot as plt

base64logo = \
b"iVBORw0KGgoAAAANSUhEUgAAA6AAAAD+CAYAAAAkqWirAAAABHNCSVQICAgIfAhkiAAAAAlwSFlzAAAXFQAAFxUBNdt0LQAAABl0RVh0U29mdHdhcmUAd3d3Lmlua3NjYXBlLm9yZ5vuPBoAACAASURBVHic7N15mBTV2Tbw+6nqngVcQDQqgxEVtwDGLW5RWcTduCVO9pjFiIkJBmSme3rAlGF6HYKRV02I75LErBPjEo27DGrUxOhnIuCuqAHUuGFQZunuer4/BowiTFX1dFV199y/6zK5YE5V3QzDTD99znkOQERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERERETedY8f3xB2BiIiIiIiqlyRsANQdburab8xhkQ/LapfQUFuApANOxMREREREVUmFqDk2Z17HrK92dd7hkDOAXASVCMAoDY6Q45GREREREQVjAUoudI9fnyD9I883hY5x+jr+7RCRmw+xjQKj4eRjYiIiIiIqgMLUBrU0rEH7Csofg15OU8FYwSAbnlo76urJz0PPBFoPiIiIiIiqh4sQOlDbpkwoX5ET+PpCj0fsI8DRFxc9lQzfl/0PRwREREREVUtFqD0nu5dDhwvZv676JGvKnQHL9eqCJffEhERERHRoFiAEpY2Tfw4IBcDhc8rpKSvCVGbBSgREREREQ2KBegw1j124tEQiQE4FYCbZbZbxRlQIiIiIiJywgJ0mFFAusdNOksUlwD4eLnuaxbZfYiIiIiIiAbHAnQY6R47ecYyaFoUh5b51vmRY+ufxctlvisREREREdUUFqDDwMalth2ATvHpEU8d+sgjeZ/uTURERERENYIFaA27e+wBBxti/xDAVJ8fxeW3RERERETkiAVoDbqrab8xEY1comJfCMD0/YECNiAiIiIiIiJHLEBryMOHHBJd/2rft6G4VAXbB/VcVRagRERERETkjAVojVjaNOnU9a/0/QjAhKCfLYbBApSIiIiIiByxAK1y93108uhi0c4ocH5IEQo99RueCenZRERERERURViAVrFlTZPPKRT1SkB2CjHGs6c8+2xfiM8nIiIiIqIqwQK0Ct27+/67FvPmlQo9K+wsAu7/JCIiIiIid4ywA5A33U0Tz7WL5pMQhF58AoBCWIASEREREZErnAGtEg+MO7Kx316fUWCWathp/kPUZgFKRERERESusACtAt1jJ+/Xp+u7IJgcdpbN2TwDlIiIiIiIXOIS3Aq3tGniVyD6MFB5xSeAYoNs93TYIYiIiIiIqDpwBrRCdY8f34D8yB8D8tWwswxi1VGrH+wJOwQREREREVUHFqAV6K6m/cZIf+Q6FRwbdpZBia4MOwIREREREVUPFqAVZulHPzZRinKzCsaHncWRjSfCjkBERERERNWDBWgFWbbbxE9oUW4BsGPYWdwQ4REsRERERETkHpsQVYhlTROnqS13o0qKTwCwbRagRERERETkHgvQCrB03ORTFHIrgG3DzuKWAnbelifDzkFERERERNWDS3BD1j128gyo/gFAfdhZvBDBiye++ti7YecgIiIiIqLqwRnQEC3b9YBjRPRGAA1hZ/FMweW3RERERETkCWdAQ3L3bpP2Utv+A4ARYWcphYjwCBYiIiIiIvKEBWgIusfusyNs3Apgp7CzlEqVR7AQEREREZE3XIIbsK6JE+sgdTcA2DvsLENhi80luERERERE5AkL0IDtuM74DoBPhp1jiLS+t8AZUCIiIiIi8oQFaMAEen7YGYZKgH8e/fpT68POQURERERE1YUFaIDu2mXixwDsG3aOoVIVLr8lIiIiIiLPWIAGyDTlrLAzlINw/ycREREREZWABWiwTgo7QDmosAMuERERERF5xwI0IA8fckgUwCFh5ygH2+YSXCIiIiIi8o4FaEDe+VfvgQAaw85RDnUR4QwoERERERF5xgI0IFrEYWFnKAeBrD3mpeVvhZ2DiIiIiIiqDwvQgKjIxLAzlINCV4adgYiIiIiIqhML0ICIyh5hZygT7v8kIiIiIqKSsAANimiNFKDc/0lERERERKVhARoABQTA7mHnKAdhB1wiIiIiIioRC9AA3LHzASMANISdoxxUejkDSkREREREJWEBGoBGMUeEnaFMXp229unXww5BRERERETViQVoAIrQkWFnKBMuvyUiIiIiopKxAA1AEfnamAEV7v8kIiIiIqLSsQANgGmiLuwMZWHb3P9JREREREQlYwEaALUNM+wM5SDCJbhERERERFS6SNgBhgOJqMAOO8XQ9RXMlWFnqEWdnZ0j+1X3NorFanmj4i3TNDeIyPqWlpZ3ww5DRIPLZDLbFw1jL6NYlM0/JiKFurq6p+fMmdMTRrZKZVmWYY4YsVekWNxuSx8vRqOvz5s798WgcxER1YIP/TCi8rt73MTDDJW/hp1jiN6YtmbFjmGHqDUdmcxcgaQARMPOUqJXoXgaos+oyKNSMJYlEnNXioiGHYxouFNVSWY7cwKdg8FXPPUKdE4iHv9xUNkqWTKXOxC2Xgtgr0EHCu7XfPSz8+bNWRNMMiKi2sACNABLx33scFHjL2HnGApR3Dt17YopYeeoJQvS6emGGHeHncMHrwNYKoLfbNPYeOusWbP6wg5ENBwl07nzIHq1y+G2Co6cF4s95GuoCmdZViTa0PgEgAkuL7mzPR47wc9MRES1hntAA2CoWfWfZ+X+z7IzxTg17Aw+2RFAsyquX7+h5+VkJveTVCq1X9ihiIYd0bM9jDagWqvfk1wzGhr2hfviEwCOy2Qy2/uVh4ioFlV9YVQNbLWr/vOsAnbALTMbGB12hgCMBnSmGubKZCZ7fSrVeXjYgYiGkTFeBsvw+J40qIhh7ODxEiNvml6vISIa1qq+MKoGUgNLncXmGaA0JAaAM9Ww/9KRzt7UsXDh7mEHIiIiIqLgsQANgG0aVf95zpv9LECpLERwmhSKK5PpbKyrq6taOv8SERERURlUfWFUDQzVav88rzvhn0+tDTsE1ZSREGSeeX7Vnxd0du4RdhgiIiIiCka1F0ZVQVSregmugA2IyDdHGEX7b+l07qSwgxARERGR/1iABqBY5XtA2YCIfDbGFr25I529IOwgREREROQvFqABMAENO8NQqLIBEfnOFMGPOzKZS8MOQkRERET+YQEagKJhFMPOMCSGvTLsCDQ8COSSZDY7P+wcREREROSPSNgBhoWiFqp5Ea5Z5AxoxVBcCqAnpKc3wsA2qthBFBMg2B/AjmV/iuIHyUxuXXu89b/Kfm8iIiIiChUL0ACIYRdRvY1w3zl27YrVYYegAfmGuh9Zs2evCzvHJguy2X1N1ekAzlbIdJRtVYVe3pHJvDAvHr+pPPcjIiIiokpQtVVRNRGp5iW4slKqfA8r+Wd+LPZUIh7/cSIePz4i2F2hKQBvl+HWIpCfp9PpPctwLyIiIiKqECxAA2AU7aotQBXK5bfkSiwWWz0vHm/P19eNV0gOQH6ItxxdFOPaxYsX15cjHxERERGFjwVoEAyzEHaEUhnKI1jIG2v27HXz4q0xsc0DATw8lHsJcND6np54maIRERERUchYgAYgX6jeGVBbwBlQKkkiMffxfG/PJwX40ZBupGhLpVL7lCkWEREREYWIBWgQzEjVzoCaBYMFKJXMsqz+RDw2WxXfAlDqGzH1aphXlDMXEREREYWDBWgAxO7vCztDKQTY0P3qYy+GnYOq37y22E9U5YsovQg9PpXLTSlnJiIiIiIKHgvQAGjR7A07QylU8IQF2GHnoNowr631dwL9Vsk3sHFJGeMQERERUQhYgAagv9DbE3aGUqhy/yeVVyIevxrQy0u5VqHTU6nOI8udiYiIiIiCwwI0AO/sWl+VM6AiwgKUym7H0aNbAPyllGvVKJ5X5jhEREREFCAWoAFoXrmyH6XvfQuN2jYLUCq7mTNn5m3BVwGUsDdami3L2qbcmYiIiIgoGJGwAwwjvQBGhh3CCzG4BJf8MT8We6ojk7tMoF7P+Nwm0th4BoBf+ZGLiIgG19XVZT7z3Kr5EBwPSD2gz9iGXDK/tfWZsLMNJx2ZzBkAvi0wxgz8jq5TlavntbX+LtRgRC6wAA1OtRWgva+t/tgqYGXYOahGFeqj2Whf/wUARnm5TlROgc8FaEcm8ylRORxGlX+PtGErZHmhb8PvLMvytaFYKpX7pIqeCAN1fj5nC9ZD5R0o3lXgbYj9ciESedKaO/f1gHMQDQvPPv9CGwTfH/iVAsAhhq2fWLJkyf4zZ87Mh5nNLVWVdC53hq16iIhEP/gxUYH9aL6391q/v2+WakE2u68o/gDA3Ph3AAAQ0eM6Mpl/zovHHwgv3eA6MplPAThCRMwPfMBGAaIr9t5zz983NzeXfdXgglxuL1H9lAC7bPo9VS0KsHzj33VFH5loXXbZqEh//5cEGLeVIe8WVW+4JB5fHmiwElX3i6vqUm2NiJ5sxu+rbtkwVQ9r9ux1yXT2xxC0ebtSj1NVERF1HutdMpudD8UPIHj/z/XqJIBAEW1snA7gm349JpXNnq6q1wMwwvmc6cY/KwAIooUikpnsmxA8BZXHVHFPnYnu1tbWV8JIR1QrLMsaodDvbeFDe72xbt3nAFwTdKZSpDK5SyGYL5APfZ8XKABBtL7xKgAXhpHPianyWYWaW/qYwPgSgIosQFOZ3CUKvRTAh3++ysD/PPP8C1MBXFDO5y5Ip6catv4JwIgPPnLgp0a0ofFCy7KOsyyrv5zPLZdMJvPxYl//9QD2GGycCZnXkckcPy8evzegaCXjHtDgVFUBqiJPhJ2Bap8dMa6G96N+dl6QzU7yIw8AQMv7g68iKL5qWVaDb7dXOQ+V9/NkByiOBHSmiP46b+vLyUxmZSqdvbwjmz0i7HBE1SjaMOIbAMZs8YOKFlWVYBOVRgRnOg/CTKuz8yMBxPFMoWcN8tEzLcuqtO/HWLx4cb1CW51HqvPfjUeGmAuxWfG5maPrGhq+Vu7nlkMq1XlkEbIMDsXnRnWAfNXfROVRcV+gNWxD2AG8EGUDIvLf/JaWVQD+7PU6U41P+BBnk9E+3jssEYwcuZ1/t9cq+ZzJx1QwSxQPJjPZp5LZ7Px0Oj0+7FRE1aCrq8sE9KKtfVyByalcbnqQmUqlwFoXw8xIQY/zPYxHG79nHTjIkF3r6kZU3JFlb7/77pFwsRVNgbKuVBl4U0Q/7jhO5NByPrccFqTT09Ww74CHrUqirr62Q8cCNDhVVYAqj2ChgAj0Vq/XqGBfP7LQsLIPFD+wxXiuI529KZ1OHxJ2IKJK9uyqVWcB2GvQQYqLg0kzRIpuN8NE7Bl+R/HKhjnI7OcANQebIQ2HAcPV59IA7i7ncy+99FKBiy2HaiPqNCZIqVzuNEOMPwHw0vl/VT5q/sivTOXEAjQ474YdwAuxWYBSMET1Ts/XAPv7kYWGJUMEp9li/C2Zzt7IQpRoyxSY42LYST/IZCb6HmaoIobLNz7lBH+DlMJFcan4dMUthxa4mk1WwW1+R6l0yUymWW29DoCXrTNPmdCp1dKAjwVoQARSNTOgAvRj7Zhnw85R6wRSH3aGSjBy5MgVADx2n7MHfxeePqShWCzh3NVhRSA43Rbjbx3p7C9SqdROYQciqhQdmczRA3uqHYkBme17oCFKzJ27HNA1LoaOW5DNVsyKG2vhwh0hOMrF0PGpzs6DfA/kUjab3RaAmyWuPY3RqOdtObUkmcl8CZBfAZ5mZP9f3jSOjcfjL/mVq9xYgAbEluqZAVXg6WlYVtHtqGuBwt5aK+1hZdasWX0Anvd42fZ+ZKld+td4PP522CmqhIjgy2qYjyfTua9X3CwCUQgE4npprQBfyuVyuziPDM/GLuq3uxoLHO9zHNfqisUzAGyx++3m1LYrZhluEZgGdydvLJ0zZ05VNe0sp1Qm8y1Afg4vp5Qo7jOh062Wln/5l6z8eAxLQARaTTOgXH4bABHZruqP+SgXxasQ7OPhim19y+LOW4B4LZrD8kTE81E3vigAWF+me0Xg/9fAjhD9n2Qu92lr4cJzq2VZE1G5Lcjl9oatp3u4pD5f1G8DuMSvTGVyO4CvOw0SG8cDuML/OM5U4bqoFOBsAPN9jOOejePg6q08cfWmQC1KprMxBTJerlHBLSPqop+pxqKdBWhA1MYGqZL30VVZgAZBVVXcfUeueQq87e0zIdv4eRaoC3e3x1vPCenZ1eq+9nisrB0yLcvaBtts01DX37+jGsYeKrKXqE4A5CAAnwDQONRniOKUaKH4aCqX+0KitfW+oacmqi6G6hx4XTEnuLCzszPb0tJSsau/DNU7bZECnF4LC6YtWbIkOnPmzHwwybbMsqxtAHf7KAfIx1Kp1H6JROJJ30K5pKIz4OL1jm0Mz/2fyXTWguD7Hi/77U6jRn0l7K/LUrEADYhA33Xzj68SCDvgBkIgb4adoVKISPHDp1ITDc6yrHcAvAPgdQBPbvaxukhDw6EiMkUV5wgwlP1Q49TWpR2ZTNu8eHzhUDITVRNr0aId0J//cgmX7tBXsL8M4CflzlQubW1tbyUz2YcAxz2V277xxtuHAbg/gFhbFakfcTKg3s5zNiJnA0j5k8idZDK5MyBuGgeumt/a+ozvgSqIqko6m1ukwPc8XSj4Sb6n58KZM2Nez1GvGCxAAyJibNAqeYFtm0UWoAGwVdcY1TIt7jv1ekblhhBnP6kKWJbVD+CBjf+lU6nUPmqan4XqTECaSrhlRCCdyUxmXL63d45lWVX7g5/IrWg+/124OLtxS0Qwx7Ksn1byvxWB3K5Qx6Y+auoMhFyAiuBMr9foQMfcUAtQmObxcDUDI7f4nqWCdHV1mals9qeAOC4D/wBBtj0Wi/sUKzBsQhQQrZ49oIXeun52wA3AwKwfAYB6OGR54/h3/MpCtSmRSDzdHostyPf27gnouVA8Vtqd5KJoQ+OvLMuqK29CosqyePHielVcMIRb7F3X2Hha2QL5wBZ1u+Qz1EZES5YsiQJ6cgmXHrqgs3OPsgfyxt3xK7CHzf7PJUuWRJ9dteo3HotPFWisFopPgAVoYASolgL0mVOefZbHNVCgBOrpWBUBVvuVhWqbZVn97fH4LxLx1gMB/bLLoxg297loQ+MNLEKplr2zYcNXBRhSN1tVuO6eG4ZCT8/DAjh3D1UcnslkQuu+/vpbb00HMLqUa8W2zyhzHG9UprkY1V/o7e32PUsFWLx4cf3rb63rUoWXPhJFgc5MxOM534IFjAVoQGxoxW7E3wyX31KgOjoWNQHidQnuC35koeFDRLQ9Hv9lvrd3P4Vk4PksWpxc19j4M8uy+HOUao6qikIuKsOtjk2lOg8vw318YVmWDcWdLoZGbJEpvgfaGhHPy2/fu9RD59xyW5DL7Q3B7k7jBHrvxj39Na2zs3Pk+g09NwGellP3A/qFRDx+tV+5wsAfnAGplhlQhbIApWCZhSM8X6MYVo0KyD+WZb0zL97apoJPAnjOy7Wq+Hy0vvG/fIpGFJp0LvcpAG4axzgz7dlluY9fDHfngarqDL+jbIllWQYUQ5nFPDqZTO5atkAemLbt6nNmuzyTtZpZl102qr9o3wFvy7k3GCpntMfjXX7lCgsL0ICIGFUxA2oongg7Aw0vIl7ayg9QGH/zIwsNX/NisYfy9XWHAnq9pwsF3+7IZOb6FIsoFOVcOquKz6TT6T3Ldb+yKxZvA+CiUZIRyj7QurqRhwMYSgFpiGl6Oce1bNTl/k8pmjV9/IrV2fmRaF9/N5w7Lr/fOrHlhLa21pr83LAADYhdJTOgtnAJLgVnYPminur5wqL5kA9xaJizZs9el4jFPg1Bp5frBJJO5XLH+JWLKEjpdPoTAI51HKhICvR8F7c0bZHvDDmYTxKJxGuAPOo8Uvfr6Pjhbv4n+iDb0JKX326ikMCX4Q78fHe1bHl1IjF3pe+BQpLL5XaJ2vbdAA70cNmrMGRaItEaaudlP7EADYghdjUUoMUG2e7psEPQ8GHW108F8FGPlz01b96cUhrHEDkSEW2PxVqhaIX7w2kjatu/SaVSO/mZjSgIthguZvT136ZoZ1ss9t9wtXVHzrMuu8xTt/NAqbtuuEa0EPgyXIGWo4nQ9HQ6XVITo1LV19cfBGBHx4GC22r1WLWOhQt3z9t6HxSTXF+keNE25Jj21ta/+xgtdCxAA1LIoxqW4D5/1OoHe8IOQcOHIcY3vV4jwJ/8yEL0fu1tsU6BJtxfIU1qmNeoKg/3paqVTqfHAzjbcaDIkng8/raIKNS43MWtt4309Z031Hx+EVPc7kEMdBnuDzKZiQD2LcOtomoYgR6JoyLult/W6P7PVCq1nxSKfwYwwcNlT0QMHD2/tbXm+1ywAA2IiF0NBeiTYQeg4WNBNrsv4KkN+QBD/lj+NEQflojHMxD80MMlJ6Yynef6FojIZwpjNoCIw7CCqXrFpl9sO7Lh5wBedbq3AN8bOM+y8vRv2PAggHWOAxXHBfkmUwSG8+ynIAvFfU7DVDXQZbi2u/2fhf66urt8DxOwH2QyE9UwlwIY5/4qeUTs4pRYLDYsjpljARoQQ4oVX4Cq8mgLCo4JXDrwf54819bScq8feYi2JNHa2gLgBtcXiHamUqkx/iUi8kc6nR6tgq87jRPB7+Px+Eubfj1r1qw+hS5xfoI0vf7WW97fdAyAZVkFQO92GqfAR1KdnR8PItPG5znORhu2/VMR/Y3z3eSkzs7OkeXI5cSyrDqBfNLF0L9Ys2c7F/5VpCObPdSE3AMvjaMU9+R7G6YP7EceHliABqTeGFPxBaiIOh/GTFQGyWz2OFV81vOFgv+p1b0iVJlERPP1dV+D+7Nnd7QNM+1jJCJf2IZxAYBtHAcWP7zktmCaVwLodX6KlK27btmp4WofqLo8WmSostnsOEAPHjQL8I+2trbntVi8Ds6dfBv7CnpS+RJunVlffxQA52JXUFMdXlO53BRRLAXg+k1IVdzcWB892bJm/dvHaBWHBWhAjlz9YK+6avMdJr6wJ/9Z1uLtoPhJCZe+mzfNmjqImaqDNXv2OrGNLwDIuxkvwDeSudygLxyJKsmSJUuiUHzbcaDivkSi5a+b/7bV0vIvQH/t4lEHL0inp5YQ0XcRQ2+Du8ZjgewDzQ/Mfg663Fd0YHVGe3v7qxA86HRPA8EswxWX+z8N266ZAjSVyp2itt4KYFu314jiVzvtMOrsOXPmDLv+KyxAAyKASsUfxSINYSeg2ldX33MVvG3KHyD4iTV37uvlT0TkLJFoeRCCxS6HG7Dteb4GIiqjN9at+xJc7FdTGJdt9YNF8zK4KOAERkXOgsZisdUQOB4HIpBjFy1a1Oh3HlE4Hr9iyvvOLbbheIaxCk63LMv313ouC9DX+vr6XBx/U/k6Mpkz1NDrAHj4upAr+/t6vjJz5kxXb2zWGqeN5lRGAmxQN8tbQqLQprAzUG1LZXLzFPrFEi5dHxVZWPZARB5EgEsLwOcBjHUeLWcmOzsPaG9pecz3YGWiInt3pHMVuUcvMDb2DztCGGzFRS4666zaZ6/dt9oErr29ZUVHJnOnQE4Y7CYiOHVBNrv//FjsCe9J/aUqtwnU6ciMhp58/igAjntGS5VKpcYo4HS28AvxePwfm35hwP6DDaMTg8+abltXN2I6gFvKkXNLstnstgXFoU7jVHGbZVkVvjLQWUc2+0VR/AxeaipBtj3WGvctVBVgARogdbl8KyziqVsXkTepTGamQheUcq1AO1pbY6+UOxORF7FYbH0qm42r4hcuhovYdgLA5/zOVS6iOAkSzB6xSjUc96Gk07mTbKhjYx0VLGpubi4ONkaARQAGLUABiAF8D8BMDzEDoSq3i6iLc1BxPHwsQFUinwLU4TW6fmDGs62t7YWOTPbvAhw06FWmngUfC9C86lSBOHc7Nqr/+JWOdPYCUVwJ9ytKFYqW9njMS3f1msQluAESoC/sDA5YgJIvUplMq0J+XNrV8mR/b++PypuIqDT9PT2/AvCUm7GqOKej44e7+RyJaEhUbDdLYt8q9PT8zGlQezx+OxTOs/6Kc5PJ5M4unhuoYt+79wJ4x3Ggir+NiMR5+a1uoTu3iPMyXCjOtCzLtwkocXf8il0wjDv9yhCEZDobE8GP4b6WKkLlvPY2Fp8AC9BgiZsOcaEa342pnBWnslm0aFFjMpu9WiFZODRT2IqCip5rWVZ/ubMRlcKyLBvqejm4YUTypSw5JwpEsrPzAIXzfj2FLLEsy7kwAyCGq73S9YhELnBzvyBZltWvimXOI/WgVCq1k08ZRgDq1Ojo9UJv7wOb/2ZR9ToXj9jRrK8/urR0zgRuinN5ZKBxVXVKprMxCDIeLulXlc+2t7X+r2+hqgwL0ACpotJfRDditzf3CzsE1YZkLndgT1/+L1CcV/JNFOl5sdhDZYxFNGTbjmy4BsDLbsYqjHN9jkNUuqK2wPnNwTwK5lVub7lNY+MvFXDeMqG4MIhmPl6JqJvOrIYaxjQ/nh+pH3kigBGDjVHgxoGzSz/oknh8JSBPOj3DhOFLN9xkMrmzAh9zHKiuPscVR1Ulmcks8lh8vmuofGpeW+sffAtWhViABkoqfQku1C7y6AAaklQqNSaVzl4OW/8GwQGl3kcFt+291x6XljMbUTnMmjWrTyE/dzda9+vIZg/zNxGRdx0di5oAbXYeKb+bN+/if7q976xZs/pEXB21tVNPX+HLbu8bFNs03e2PFPHpOBbbsTg0DPnQ8tv3qLrohqufVtVSViUNft9IZAZcrHZSd0V+Renq6jKT2dzVgMz2cNlbYhvHt7W13uFbsCrFAjRQWtFNnYllVgAAIABJREFUiABAICxAqSQdCxfunsxmc2qYL6hgFobW5Oxp07a/4NTwgigsEdi/dTtWgM/4mYWoJJHC9wDUOQ0ztOj2+KH3SLF4FQDnsw1FL7Ysq6Jei85vaVkF4BnHgXb5zwO1LCsiglMchr3Tv2HDXVv7oAHbeR8opCmZy33CYzxn6mr/51uF3t6qWtlkWVbk6edW/Z8A33B7jQKvwDSmJhItjuezDkcV9Y9+GPjQcomKo/hk2BGoeiSTnZOSmdxFyUz2HikUV0HRgiEfNaRrDLVPbGtre6ssIYl8EI/H/yFwPjMQAKAy3ec4RJ5ks9ltBfpNp3ECWdbW1vY3r/dPJBKvKfBLF0P3qRsxwqngCoGLGTrB7gtyub3L+dRoY+MUAGMGfazgVsuyttpTJB6PPwzgJceHqZR9Ga4AbpYl37ml5cOVavHixfXRhoYuEXiZrV9lqn1MNR3DFTQ2nAlW5f+DExx0yw4TtjvlzWf/HXYU+rC6vv7vJ7PZ0JZyi42RKrodgH0B2RewR5X5Ea+JHTmhLTH3hTLfl6jsVPFHCCa6GHmQtWjRDtacOW/6n4rIWcHG+RBs7zTOhr2o1Geo4Iei+AacJjtsXAzg5lKf45PbAXzXaZBp2zPgZrbULZWznA4DUtWtL78FICKazGSuB+SiQcdBPw2gzXvILVuQy+0NW8c7DlSpmuNXOjs7R67f0HM94GW5tT6uhboT2ubNWeNfsurHAjRIKgVIxZ8yZjaOqD8Kb6Lq1ucPBwp8L8yD6gZ2jJR928gmLxehJ16SmPu4Xw8gKicDcq8NdfMCzqjL56cCcNOhkshXlmVFMLBNwskzhd7eP5X6nPmx2FPJTPZ2ACcPNk6hUzuy2cMqqeFcvre3O9rQ2AugYbBxOnAeaIlHjG12L1VJ5XKnO/yMzxuqtzrdSwzjerV10AIUwN4/yGQmXxKPL/eSc6vPLOpxLl4eqBYjVVGAZjKZ7fuL9i0AjnJ/lfxN7OLJ7fPmvOFbsBrBJbhBEq38GVAAUBnWB5FT8ARYqRHzyHL9ICQKQl9fwwMAXO1TVlXfjj0g8iLS2PhZAB91GifQyyzLsofyLFsNVzOoooPP1gXNsqwNAO5zHinTynWmZiaTORSKQc8NVmi3m+0pE8aP/7MAjsecmCJne8k4GBEX+z8Vy+dVwcygtWjRDkXIHfBUfKI7InpcIpFg8ekCZ0CDpMj7N3lUVmcqMFs2WwfSjakRjHv9aNh6oogcu7HV9noAz6rqbaZh3Dpl9XIWEOSJCm4xVb+QmDv37bCzEHlhWbP+3ZHJPibAQU5jVWT/IDINUTdUXDdXqkmGTti4l71miWKOi2FvRk3zF0N91vy2lruSmezfARw4+EhtzmQybfF43HnvYlAUt0McGw2NijQ2HgrgL0N9nO3iaBQDGHT57SbNzc3Fjkz2RgEG3eerirMADLnb/MZGUlOdxqlIxa+uy+VyuxT683coMNn9VXJjvnfD59oH2ZtLH8QCNEhSBXtAB+y+dOwBBz28a3T5+pd7DwfkuIGC8/XDoRgJkfdXpqMA7CYi02zV7LKmSS8Bem3BNK6c8dLy50P7E1A16IfAKvT0ZOcN8V12orAI5AlAHQtQUewbRJ6h0RXtbbGfhp0iTKlc7hhVrdkCdEE6PR2Ac7d7xY9bWlreLctDVS6H6P85jIoUYHwXqJziX9S8VaW40HHcwDLcIRegEJzpMEJNkZvc306vB2TQAlSAjy/I5faa39r6nNv7bkl0xIgDYeuOjpkq/PiVjoULd88XincBmOD2GlVcU+jb8PVqaqxUCbgEN0AqUjVfnIbYXetf6XsLIvdBYCl0OoCRTtcp8FGFzDGL+tSypknJLpxjBhCXqo4+DkOObI/F0kNd4kUUJoX9rMuhu1uWNejh8kR+E8O42MWwfMRwdY6nK/m+Db8G8LLTOIF9fiaTcWyMFJREYu7jULzoOFAxY6jP2thN12mVxF9jsdhqt/fccfTouwA4Ltc1ijr0ZbhFdXP8yrvbNjY+MORn+UQEe0mh+Gd4KD4h+GF7vPVcFp/ecQY0SFole0AH7OV2YP2uO79q9/bV5d9aN/p9vx1RIPGRpidGYQ0u9CHfMKLwsfFP0HqgyG07ckR61qxZoXXzLYMjUtlsV9ghNlFFEZAH9t5z/FU8OzVYhsiz6q4xmGE0NOwFgNsUKBSpVGo/Vbjo8aC/isXirgsdJ5Zl9acyuasUumDwkbJdUeXrAC4r17OHzMDtUJzvMOpIy1q8nWXNKvn0ANO2P61OP+fV3fLbTWbOnJlPpbO3qOCLgw40cBaATi/3/hA3+z8hd1X4z31v+/QF2fZYLN4ei/kUp7axAA2QAbh8nVJdDrnhly/WNe1ymOaL/+xds3bN6v++Bmt+9psjAECBb3ePm2jXY7vWo1Y/6HwoNW1BTRSfRUCvMYHvx9sqaI9P6cap4pywQ3yQfu6ZVat2BzA37CTDishLbr+zRwxjB5/TEG2VGkYL3Kx8M4zLy/3s/rrIVdH+fBxOK6lE5yxZsuSKmTNn5sudoRQC3K5wLECj0YYNxwAouWOwwnH5LWzDWwEKADbkeoEOXoAqjshms+O8zK6+n2VZdXBRvAnsquh+64KqYM68WOxHYQepZlyCGyCtgjNYSvHGPX/uAZCHKSP6Xnm177Vb7/zg7KnKd/p1/bJbdpiwXTgJKUT9qrhG7OKk9nj8axXVYKIGqTq8001lJ8XiOx6Gb+tbEKJBJJPJnQH5gtM4gd7V3tr693I/35oz500IrnExdNxrb75dts6sQ9Xf03gXADfFsIdzIj8om82OBeSwwUfp4/Njsae83rvQt+FWAE57eaWgeobXe29SN2LEkXCxRUtUa6QAldksPoeOBSgN2VOtl05ZNm5y9J6PfnzM3z/ztSn9r76+0+ZjFDiscUTD18LIR2GRX0YEe8xri30lkUg8GXaa4UBY4ASuYBiuC1C7yL8fCoea5iw4nGkJALAN35a/SrF4GQDHPf8iWjGrODYuq33QxdCSC9D8wOznoEud1GX3280NHCejdziPFMcOvFuj7vZ/PtXW1lYjjSn128lkctewU1Q7FqAUHMURYUegIOlOsVhsbdgpiPwk+ajrAtQQm6tAKHCWZY0QyEwXQ5/q79/gW5fSRCLxNCBulqkemsrljvErh1cCcTFzJx/LZrPjSrq/Oi+/NVVLKkABQESudzFsaiqV+tDkgSuGmyZMld391qN9YEa6c7ncLmEHqWYsQAMkakfDzhAmEXD/01Ao7jPU3iGM/6CuXrxs7sRkNuvmnVGiqiXS46WLM3/mUuCiDSO+AWCM40CVRb53JRd1NcOqNtx06w2EGnBVPBXhphHPB1mXXTYKwBSHBGvi8fjDXu+9SX9d3U0A+h2GmSqRT3m9dzab3RaKQ10MrZHlt+/ZN1/U21OplPO/K9oiNiEKkBoyAjW5C3RQawRYo8BhUHjZK0WbEZF8W7zNsaW6H7q6uv7nmedXXQxgH08XKhZalnUIj1oJhgLrw84w3IjItm6/rdsiJXfJJCpFV1eX+czzqy5yMfS1xvqImz2aQ9Iei3V3ZLKPCuBwdq6eviCb3X9+LPaE35mcJFpaHk1lcy8DGHTZparMAPBzL/eO9vWdBkjdYGNE5XqR0nuIWLNnr0tmst0AThxsnELPAvC/Xu6dV50qEKfJld58b+89Xu4bCsH9UEwAsLPL8QeomHek0+kZbW3hvDarZnw3NkjqvEm7lgjwAGAcMXXNisOh8l1bsSbsTFSa5ubmoghSJVx6YLShobnsgWiLBPr7sDMMNyLi+vu6wTcIKGBPP/fC2XBzrJriqjlz5gTSqd4QV8esiAHM8j2MCwPFn97pPFKPV1Wvbesdl99qCd1vNydQx2W4IjjeshZ72iYg7mZ97xnYi1rZ1MazRejxAN7wcNnBRTHuTqfTo52H0vtxBjRYNX8IuQD9tsgfRGXx1DWP/WXT709bu/yKuz46ec8ws9HQTNhjj18+8/yqNgD7ertSUpZlXWdZltMSoGqyWsRVY4pAbDwH9C87jh51VdhZhhs1zW1gu5uc4Aw1BU1E57gY1hc15Se+h9lozKhRv339rXUpAIPvmVScay1cON+aO/f1YJINEkWN20T0Kw7Ddl6QzU6Cy7N+LctqAOQEh2Hrdhw16l5XIQehxeINMCNXAjAHGVYfbdhwCoDfur6xyHFOK/tU3C1hrgSXxOPLk7ncDNi6FICrolKAg2wxbslmsyfEYjF+j3eJBWiwtg87gI9eFdElhmn/ZOqLT7y8pQEzXlpeIx3Qhqfm5uZiMpNJAeJpiRGAPSKNjecDuMKPXCH5SyIW48wuQW27yfVZvSJv+5uG6D9Sudwxaqtj8z8Fftna2vpKEJkAYObMmflkNnsV1HFVTWO0WPwWgAVB5BpMoT5ye7Q/X8TgBRyMgW64rgrQusbGE1SdOmPrH8txJmp7e/uryUz2QTie1ylnwWUBanV2fgRFe6LTOKNYrJoCFADaW1v/nkp1nqqGfQeAbVxedkRBcZtlWSdalsXtZi5wCW6wdgs7QNkJHhHgKz2NvbtPXb3y+8dupfik2rD3nnv+CoDns8hEcYnXpT1E1UBgTHA7Nr+h4Vk/sxC9n9tGPmIai/3OsjnDtn8COPeFEMV3BmYKw2XNmfMmoI6NgATioiPsANtF91tVc8jLb9+7l4tluABOWbRoUaOb+0WLxRlwfvfthWo8hi2RaHnQVvsMAF6WpR8VbWi8tbOzc1httysVC9CA3LnnIdujhmZABXjANmTKtNUrDp26ZsU1pzz7bF/Ymch/zc3NRUA7Srh0p2h9j5ulYERVRWE7768bsHbjmYJEvluQy+0NqJuupre3t7Q85nugzQw0bXFeTaPAR+oaG78YRCZHKm5m8qYsXry43mlQV1eXKcBpDsN66iNwcYanO6bqdYBjK8xtevN5t2eaOu//rKLlt5ub39a21FA5E4CX17dH54vFGyrhTZNKxwI0IGZv76SwM5RJUUTPn7JmxdHH/XP5kPclUPXZe889fwOI93c0BRfz3CyqOSJu90R7XjlAVCpDdQ5cvcZzdyyKH2xTfgig6DROFS2WZYX+elXUcHOUyIi33333SKdBTz333DEAHM7dlDtaWlredZfOWVtb2wsAHnUap6pnubujTHccUeXHr7S1td4hgs8BKLi9RiEzIg0NN7IIHVzo/6CHCxHjpLAzlIf8curqlVeL87toVKOam5uLKiXNgm6TL+r8sgciCollWQ1QHOxqsLAApWCkUqkxUHzZcaBgRSIWK9sMm1fzW1pWAbjJxdB9ow0Ngx4hEoQJE3Z/CC46pIqI4wyiIeK4/BY69O63mxOIi2W4cqZlWYMeDZNOpycAGO9wo7yh2u06XIVKxGI3iODzcPFmySYCOSHS2Hi9m9nw4YoFqA8UkNvGTdzh7t0m7bV018mHLBs78VOAfivsXGWhel/YESh8++yxx28BeD+fTXB+KpXar/yJiIJXN2LE4QDcvcCw5a/+piEaoGJ+F3Bx7Jstlw3lfMlyUNczsOJqP6ufNh5HdpfTOIHhYgmrnO4woJiPGje7jOaebVznYtSoaGPjlMEGqIjj8luB3B+Px2ui8VoiFrsWKucBcH2muShOWr9hw2+WLFnidE7qsMQCdBDd4w8cdc/OB+xx99gDDu4eO3nGsqbJ5ywdN/n87nGTY91jJ2aWNU1c0t00qau7adKd3eMmPbysadJz3U2T3lrWNMmuV3nDsPGsGPqwivwRwJiw/zxlIeL6HSCqXc3NzUVVKaUzYQSm+YOyByIKg41j3A41xV7qZxQiYOOsvOACp3EC/Cvft+HXQWQazLx4/F4AD7kYelw6nT7I7zxO1HazD1QPthYt2mFrH03mcgcD2MPhJvf6cfxMIjH3cVdbaFQGX4brogBV1ard/7kl7W2tP1PBRd6ukrNef2vdry3L4qkjm6n5T8idex6yfWRDcbRh9I8uiIwWwWgBRostoxW6A0RGCzBagdFQjMbGjyswGvmC2JH/VOkK4L33CkU+uAZVh8eaVFV8NOwMVBkKfRt+F21sTEDhaX+zKj6TSnUemUi0VMw5mkSlUNinuTyC5Zl4PP6S33mI6hoazlVgZ+eRcqVlWb3+J3JDLwfkV06jijBmA3A6i9NXURO35e2Bl4ODDDPr8vnpAK7d4keLeqbTtw2V8i+/fe/esK8TSMJh1NmWZX3HsqwPzfhZlmWoYqrTc0yprQIUAObFYlck07koRBd5uOwz0YZGu6ur6wsDjRwJqKIC9N7d99+1WDB2FmC0LTJabIwGMFoEo20MFI3QgV8rsAM2FpHS12fABGwYA4XkxipRBdj0/eO9wnHjN4ThUEiWSoA3w85AlcGyLDuZySQB+Y3HS0UNOwNg0CU+RJVsQWfnHijah7kbLZz9JN+pqqSyOTczNH1azC/xPZBL+d7ermhDYxoY/A1uEXyuo+OH7fPmXfzPgKJ9SGtr6yvJTPYfAA4cbJyqzMBWClARnOn0OlMNw83e2NKIXA+FQwGKnevqRhwJ4P7NP1BfX/9x26GBkgKvxGKxx+Lx+FCSVqT2ttbLkunsaAi89LRofvq5VT2WZX19S0X9cFQ1S3CLBfMhQB5VyFJR/AGC/4agU4GEAN8C8DkITlTgMAATAIyRKvrzVYmCAb0x7BBUOfK9vV3i8tDtzRybzGROLnsgooCYxeJn4XL6UwyUfy8X0WaS2ezpAPZ3HCj4eXt7+6v+J3LHsqyCQK90MTQq0cKFvgdyoHC1DHeLTZMW5HJ7KTB58GvlkY0NmnwxLxZ7GIDj/dXccjfcoovltwBuDXt/sZ/a22KXQJD1co0Izo02NFxdCR2dK0FVzIB273bAJNj2uLBzDHcC/HTK2hWhvfNIlceyLLsjnUuK6G+9XiuQrGVZt/PdQKo2lmUZCuNcN+tlBPjXmO23r+qjCKg6CMTdWcuKGclM7mGf43ii0BEuB55vWVaHZVnv+Bxp6xG0eLuI4TS1Nz6dTu/Z1tb2/Pt/U2z7LKf3rQT+Lb993zNuVOB7gw5SfFpVWz5USIoc5/itT6Xmv+clWlvbUpncthB82/1V8vVoY6OtqufXcoHuRlUUoGrrCa7eZiY/PRXpy8cB4IFxRzYetfrBnrADUWUo9G34fbS+MQHBAV6uU2BytKHhSwB+4VM0Il/UNTaerqpuuzn/eubMmXlfA9Gwl06nP2EDx7ocvmcVbzYaHWls/DqAxWEF+MgOO9z/+lvr3gaw/WDjdOA4lg8sdRaRM50+9VoU3wtQG3q9QAYvQIHxmUzmQLzv7FDLsupEHZuvFQ0tOHYLrnYioqr6nWQ2FxXgm64vVJyXymbfhdMbADWuKqaBRfWEsDMMcy/advHEo19/ar0C0od//2bpuMmnhB2KKoNlWbZCSjkXFIAkFy1a1FjeRET+UkWL67GGXONnFiIAUMNw/TVZ7UQxO8yuohvfUHI831IhM97/62QyuTMURzhc9mx7e8uKoeRzo9Db+2cFXnEaZ8P4wDLcSEPDEXA64kfwUCKRcDwvtRaIiO6z5x7fEoHHXhhyUTKT8dLIqOZUfAHaNXFiHcT1u3pUfg8iqkcc9/ITLwLAsrGTLobKGaJ6+U1jD3G3ZIZqXnu85VooHivh0nE9fXkPy1eIwpXMZo8DcJS70frX9tbW/+drIBr20un0eFUMfmxGbRkfqR8R6p9XFW6WmB7X1dVlvvcrI3oGAHPrwwEIrh9aMncsy7JF8EfHgaKf3ux33Oz/rPnlt+/X3NxcnLDHHl8Wwe+9XSmzOzK5Tn9SVb6KL0B3flP2AsAZkuDlAenYdpf6KdNeWPkKAHQ3TbpA/7PpesI2Rq+nDdhUu0RExUAp54ICgvbBzkwjqhSWZUVgw/W71mIYJa4MIHJPB44nqYotVeUi4n4Vgi+KkT+5GDX6ueeeO/i9X4me6XSBqvq+/HYTwxYXxa58LJVKvbfdQDab1d0SBW4dWrLq09zcXOzv6fkSADdfF+8R6NxUJneJT7EqWsUXoGrC7T4bKp9latoHTVuzfP6hjzyStwCje+yk7wP48Qc6C6tcuLTpADfvhtEw0Nba+gcF/lHCpaMj/YVhs3yMqle0YcSFbvc6K/CPtpYWTy9GiLxKp9OjVfD1sHMETz+RSuU+GdbTNx4F84TTOBXzeADIZrPbApjmMPzVQm/vX8oQz5Uddtj+bgBvOQ40ImcD7/0ZPuEw+vVCT09FNbgKimVZ/Y110XMEsszLdQq9NJnNtvsUq2JVfAEKZQEaFAH+n4qcOm3NimnTX3p8JQDc9ZFJO08ZO/lPEFhbukRg/7x7/IGjgk1KlUhE1BD8oKRrod/r6PjhbuXORFQu6XR6PKCXuh1vCDqGe5dD8p+K+S0A24SdIwxq2BeHnMDFcSw4HgAKqqcCaBh8qNwQZFf4gb2s4vgmmWLgOJaiyBQA0cFHyx3DubP9nDlzeqKmnAbFfZ4uVHSkMpnaOzR1EBVfgNpuzrSiobpfVE+fsmbFodNXL78FALpwjtndNOmCSBSPQ/SkQa5tQqGwMKCcVOHaWluvL3EWtEGihWG5DIUqn2VZEVuMX8Gh6+X7PNTf03Odn5mILMuqU6ibczELGJjpqqb/Cs5/LDk9nU5PcPHn94vjXkeFHmVZ1jYi4rj81lD/j1/ZnIi62XN66ILOzj1gq4sVb/aw2v+5JS0tLe+aop8C4GkmWCHpZDo7bFaDVfyeAYHsU8XtwivZeih+B9O4fNo/H3uv45oC0j124qdEnrQAHOTqM6/4xtKmSddPX7OCy82GORHRjnTnpRDb+4tvxdd+kMn86JJ4fKUP0YhKFm1sTEHdNh5CAabxzeE8C0DBqGts/KIqxjoOFFzWHou1BhCpbDoymaRAEg7DTFvM7wH4ThCZNpfv7b0n2tC4AcBgDRnrog0Np6nqyYOf/6n/Hjmy0bGzbrn19/Tc5uLPAKNgN0McGxBp1DDuKF+66hWPx9+2Fi48ua5QXKbARNcXCrId2ey782Kxq3yMVxEqfgYUoh8JO0JNETyiIjPRr2OnrV3xzU3FpwUYy5omn7Ns7OTlInIjoAd5uy2uvm3cRDaSIbTH594Aj+/8bWSaMJLlzkM0FMlM5ktQzHU7XoDF7S0tpXSEJvLEVlzkYljBVL3C9zBlJsXiYgC9ziP1a6lUaozvgbbAsqxeAPc4j5TfALKdw5hbZs2a1VeWYB5YlrUBgPNSYkFGgckOox5tbW11PNpluLDmzn293zSmA/Kkh8tEFFd0pLMX+BasQlR+Aarg/sKhe1GBdBHmvtNWrzh0+urlP5322sp3AODOPQ/Zfum4SbOmNE16XqFdEHX/Ts0H7dpg47/KmJmqlIioCEosJPWMjkzm6PImIipNMpM5FZD/w+BTF+/3krnl/fJEZZXMZE4W4OMuhl4bj8df8j1QmbW3t7+qcHW24ggYkZm+B9oqKcuSU1UJfPnt+55elqNf1N2e2GHFamn5V0T0eACrPFwmIrgqlcl8069claDyC1Bg27ADVKlXIXqFqh49dc2KPaavWZGYseYfT2/64NJdJx+yrGnikkhf3xpRXA5g96E+UEW+sKxp8jlDvQ9Vv7bW1hsB+Vsp1wqwUFXdvuAn8sXAeZ/SBfdbVfJiG5+LxWLr/cxFNEBcNeAR2/iR30n8IkVjEVzswVLoLMuyHBr8+MOWshRdfYW+htCOLjGBmwD0l+FWw37/55bEYrHVJnQqAC9vBIlCfpzMZL7kU6zQVUMByg2g7r0Bwf+I4oTX1uzfNG31yu9OX7vyftn4OexuOmDc0rETW7ubJj0phj6skPMBjCxnAIVeefvOB3DZ9DA3MAuqJZ6BKIenc7kzypuIyL1UNvsZKG6Gw76oD1C0JRItD/qXimhAsrPzAADTHQcK7k8kWv7qfyJ/tLe3rADgZk/hztH6EZ/zO8+WzI/FngLw3FDuoYK7LWvWv8sUybN4PP62CpYO7S76751Gj+b3v62Ix+MviV08XgEvS5RNQH7Wkc593rdgIar4AlRYgDp5Q4FrRPX0bXep33Xa6hXnTV274s5m/L4IAHc17TdmadPEr3Q3TbpTYb8oIlkA+/qYZ6e6aPGnPt6fqkQiFvtjqbOgqshYllXxTdKotqiqJLPZNlV0wfHIhPdfiD8m4q2L/EtG9B9SsFvhYlm42nJZAHH8Jfihq3GGXhzayhkd2syfoRri8tsBYstQl+HeOXCsC21NIpF42oaeAOAND5eZInpNRzr3Wb9yhaXiC1AFNoSdoeIoXhHgKoFOf23N/jtPX7PiK1PXrrzp0EceyQPAfR+dPHpj0XmTicjLAvk5gBkS1N+3yhnd4yZ/MZBnUUVT2AtKvHTfaP2Ir5YzC9FgUqnUTslc7mYoUnC/5xNQPJZvqDuXZ35SEDo6FjWpoNnF0BcKfRtu9D2Qz9pjsTsB/N1xoGJSJtN5vP+JPkyMIRWgdsQwbipbmBLlI3IDgGKp1wuX37pySTy+HIbMwMBRQ26ZInpNKps93a9cYaj4AhTA62EHqBCvbZrpxNodd5u6ZsWFU9es7N4003nLDhO2W9Y06cvdTZNuLhT11Y1F52lwPDTYJ6r/dcdu+zq3h6eaNi8evwnAQyVdLHqpZVnul0ASlagjkzlDDXO5KE7xeOmzsAsnWLNnr/MlGNHmIoXvwcXPdRVcblmWi7M0K58IXO1jLYrtal9suUUN424ApXawfaASOsdaLS3/AnB/qdcbLEBda29t/bvYxqkA3vFwWVQVv0/lcqf5lSto1VCAvhZ2gLAI8JJCL1PVo+9Zs2KXTTOd07CsAAwUnUvHTf5C99hJ1zU2NryqwC8AnIqwis4PGh0tRv9XvcwkUE0SQ0qdBR0bbWx0c8wAUUlSqdQ+HdnsnwRyA4CdPV7+mtjFU9vb21/1IxvR5rLZ7LYCddNwesoZAAARNUlEQVQZc31E9f98DxSQMaNG/RrAaqdxAjkhk8m46QxcVi0tLe8KpLTiTRH68ttNVFDSMlwBVlZjp+UwJRItD0JwJlwdNfSeOrX12mQmc7JfuYJU8QWoqHppXVz1BHhJFIuhesyUNSvGT1+zcs70tSvvtwAbALrHHzhqWdPkc5Y2TfpFY2PDWlH9FQRnwct+paAITuweO/FbYcegcCVaW29GqbOgilhYZ7xR7VqQze6bzGavVsNcUcKsJwC8JLY5NZFIPO08lKg88qozAWzvNE6Aq+Px+NsBRArEzJkz8xBc5WZsEfie33m2zC5pBtA25Y/lTlKqKHAtSui7ouLiHFH6kPZY7G5D5Qx4mz2vB+TaZDY7za9cQan4AlQFT4SdIQDPAtKhthw6dc2K3aeuXXHRtLUr/7ypAdP793QiX3hFoV0CfBll7mDrB0Okc+nYA/xsekRVQGy5tMRLt4dhJsoahoalrq4uc0G6c0Yynb3RUDwOxXkoYbWIACsjgk8mEnMf9yEm0RYtWbIkKpDvuhhaFLWv9D1QwAzb/glcLVmUL2az2XG+B9qMmqbnIkyA5fNbW5/xI08pYrHYagCPlHApl9+WqK2t9Q5V4/MAvCyXHwHFzalcbopfuYJQ8QUo1FgRdgS/CPAmoF+dumbFPtPWLJ8//eXl7/3D78I5Zve4ied1j51862Z7OuvDS+ydAiNE9OfdmMqOpsNYItF6C6AlHQegwHfS6fSe5c5EtW/x4sX1C9LpqclM7opnnl+1xhD7TghOR+k/+/4sah+z8YUaUWBeW7euGcBHncaJ4Lq2trbnA4gUqLa2trdE8TMXQ6N51cBXXrW3tDwGwT+9XGNDh9p5tuwEnrvhbsj39NznS5hhYl5by/WAfg0bVzq6NEJt/VMqlzvGr1x+q/gC1LQj96M2j2K5uSjmAdPWrPz5lo6aacbvi6pyLERPQmXs6RwCPVya3mgLOwWFrtRZ0LoiDKucQaj2ZDKZ7ZO53MEd6dw5yXQ2lUxn712/oWedIUY3oBfC+x7P91MBfpTv7Tmura3NS/dCorIQxRxXA4tG9R+9shXFiLEILjq1CuRblmVtE0CkD1K908twswKOX/kQu3Ctxyu6Lcvyso+RtqA9Hv+lCM6Dt3pnpNp6U0c2e5hfufxU8bNSx77y6GvdTZOeBLB/2FnKQSBrbcW3p69d7tgeva4vf2G+PnoY/D23MxAKnd+926Rbp/1zxcNhZ6FwtMfjtyYz2T8DONrrtSL4Yjqdvqytre1RH6J5JsC+yXQ2FnYOF2yIPtYej1fCEqkDU5mMpxdog1GVeoiMAHQUgO2LwI6wFVL+tmdvKfRr7fF41R9pQdUpmc0eB8XBziPlb4lEy4P+JwrH/JaWVclM9kYAZzsMHf3/27v34LjK847jv+eshOy4JjYDMdgO4AvlYtlNsKeJSRMs21yblGZaa6YtExoaalrIxWCslWSSbUBCsh2TmEwSUgYGcpvBaZo2oaHASDYlJk1RWrAEhjq1IV7Z2C4Fy9Z19zz9xwwex1i7qz17duXv58/Ve877W0k7e55z3kvVxIl/Kenr0ad6h3vwuJnfmFtjvZpMJv+rsbG87s03NTW90tLW9qJkl+TSnvmfxdPU0PBQS9u635F8Ux6Hvddc/3p3e/sVaxsaKur6uuwLUEmS6TH5uChANw9aePPV6Z43cmn8Bwdf7uuYvuC6IAh/6a7Tow4XsWqF+s62mYsvvWzPswNxh0Fc/G7JCvnCCkILWqSCFowpOpfmy9QWd47cmFrb2jc2JRti2aLgGFNdtrxoZzOpBINjnvSqxE1rV69+NeqOgHflyvGzG26INkj8XP4Vk41WgMpctz366KPfrK+vL3hvy3xlJlQ/WT00nFEu19bmPy7XvYNd+pFJORWgiTCkAC2i5uSa++5ua6s22VfyOGyKuZ5oWbduefOaNb+KLFyRlf0QXEkKw8KWhi4bpl1yu6Iu3V1/9Z7cis+3Le194WUP7QaNj2HIFw2HfRVy0Y4oHH0S90yBh1/T0t6+rJh5ThUu3bpp06aKmj8es17Jb2hONlxJ8Yk4tba2XiTp6lEbul4dGRz8UfSJ4rU2mdwmUy5PeWft3LXrusgDHSO1atWbMuW01oEFQdle1+aRbVdjY+POSMOcgtYmkxvlasnzsKkK/fEvt7XNiyRUBCqiAF3a2/2spF/HnaMAWcm+OjwSzK/r3f5UoSep693+Y0kbi5grNm76bOfMeaN/mWLcCrzgfUElV5u7s7ds/k57Y2ho1O0boCMuWzcyOHBhczL5SNxhgDBIrFFu+2nfl0ql8llJs2JZjtdDnvOT46LKZbrD/w739xe2b2gJHH2KNupCVuZ6rARxTknNjQ1rXbYuz8POqpJ1tLZuyOnpddwqogA1yWX2QNw58rTTzZbVpbevuur1F46M9WQH0hc3SCra/KkYmdweeHzmvDPiDoJ4NDaueUKmQr98F92zbt0nihoIkPrMtUnZzAVrk2saUqlUDts9ANFqaWmZZtKf5dC0L2FeaddIBZs7a9Y/KreHEpe1tq5fHHWeYwU5DEl16Z/L/WaBSaMvkJSwclhbYNxqbrgjKdk38znGpfd5kO04OnKirFVEASpJCoceUE57QMVuxOV3DUwcrF26Z/vWYp20Xpuzw5ngeknjYfn/GTWh5TPJGuNMGBa+qq27l3RYFcYz22HyxpGa085tamz4fHNz8964EwFv80TiVkkTRmtnroeSyeRbJYhUFurr67NuyukawoPwC1HnOdbQ0FCXSftP1iawHIq7mOWwRczwcH//llJkOVWZmY8M9t8qeb6jcaZ5UPXdSEIVUcUUoHW9rxyU6Vtx5xjF8x7a4qXpni9eu3PnULFPftXrL+yXghWSin7ukjP9xZYZ81fEHSNens6ndSgfN3PR7my84ykVOBfUFfnCDcMRnz8eVVVRvq+RCM9dbAckv89NH2pOrrm4KZlsS61a9WbcoSLyWl6t3fLax3A8yprle5N3OHvkSCQ3LkzBNTk0y5rCr0XRfznLDAw8KCmHLZG8pFN+UqlUKD/paLX+4YGBgqdklUpmcHCbpHf9vzb505UyUiSVSoW5XG+ZtLsEcfKSSqXCC2bPvlHSo/kd6WU/DLdiClBJSmSq10kqxwuFAXdvUPrMRUv3bu+KsqO69Au/kHxllH2Uisu/3Tljwcy4c8TFzb6v3G8mZIMw+Pso85SaBbZW+S+u5e7h96PI8w57Otrzl55Lz0daZLnK+XfWJ+kxuVYHHl46MjhwdnMy+bm1DQ2/jDtY1EIP7lfun7HDgcLNUeapBGtXr94tqTP3I+yhqPZBdPfuHJr9Q2Nj46jz9cabVCp1WK5vjN7ScvkdFldwsnmg/rNUKtVfujCFSaVSoUzvuvVUmNtc17JhCr49SpPDYVXwcEnC5Km+vj575tQp10v2kzwOK+fvZEmVsg3LUR/b958HOmfWpuT6atxZjvFzuX1maW/3jlJ1WJfuebhjeu1CM322VH1GZIoUPujSVTY+VvnNy50NDS/ddc891wYWtMt1qeyEN4Rcru0yT463/d2a1qzZ2tre/ml3NUuao5PfEMtI/oqkljsbGzuizFUd6K9HQn1N0keVw/C3cmbSSGh6zs0iHYY2saa6fWBkZJpcH5c0Kcq+juXSkEl9kh0y6S13P6xAvQrtFVm4I3B/ec6cOa+WciuGcnJn4x1Ptba3/6lLfyfXJTrxZ2xYpi7LBrc3NjXsLnHEsmNmntqwob46E94r+bWS3m29gr0yfXdkoP+LUWVJKFzlFrzHXR+RNPG4Hw9K2mph9tao+i93kydNvKvvyMBUmT4uafJxPx42+XaFYcmvk4YTiZ9VZ7KDOsH3h5uV7eq3J/BDSTef4PXQg6Ci9kWeO/v8lv/+9a4jMv25pFlvv370O6Qr8PDO5jsadsUY8aRWrlw5cv/99//JwTfevE3mn5TsAp14cbIjZnrWM5nPlzpjvipuNclOLamyGQd/7tLvx5nDTIfkSl6e7v5WHMXTo1qROGv6jp/KSju8JCJ/U5fuLvfh1QAAAKNqbW//K3dtkvSeoy+55A9fMHv2ZyrphlhLe/sGub4gKXH0pWG51jY3NqyPMxcqX8UVoJLUMX3BhWZhl0p4p/04Pw1cf3t5b3es82UenznvjBq3f5c0N84cY2VSf0aJDy5PP/9K3FkAAADGav369ZMymcw0SXL3vqampgNxZypEauPGM2qGhqZI0tDQpIOp1OcOxZ0Jla8iC1BJ6phR+4cm/ZPeuSsTOZP2h/I7lqZ7ymZ/uK3n1F7sCf3CXafHnWVMXM9NPqfmskVdXZW0mAkAAACAPFTUIkTHWprufsxla0rY5Wb34XnlVHxK0uV7u18KQ/uUS2HcWcbEtKhv31Ap/54AAAAASqxin4C+rXPm/K/L/ZbIOjDtCkPdvKy3+4nI+iiCjhm1TSa1xJ1jjEZCDz68rPeFX8UdBAAAAEDxVXwB6pJtmVF7r6SirvjkUhjIH/Bh3V53oKci9jraMqP2YZc+FXeOsTDTi151eGHd7t2RLGsPAAAAID4VOwT3bSb5knT3Knf7koq3Gu12N1+8JN2zslKKT0ny6sMrTdoWd46xcNclPjKpNe4cAAAAAIqv4p+AHqtj+vzrAvMH/d337RrNkOR3Tz57QnulLobTOf13z5Sd9qwqeGVcl0KXX7Es3RPpfo8AAAAASmtcFaCS9PR5F5+TzSa+Idcf53noM3K7qa53+45IgpXQU2fPuySRsG2S3ht3lkKZ9NpITc2CK/6n6624swAAAAAojoofgnu8j7360t66Pd2fdLMlkp4Zrb2ZDrn7LVvT3ZePh+JTkpbv63nRXCskZeLOUiiXzq0eGtoUdw4AAAAAxTPunoAer/P9C2otzH7aZddIukhH37PJel3+g+yI1i/f3/16vCmj0Tmz9nq5HlFF/519RV2654dxpwAAAAAwdhVcmOTvX+bOrZnUP+F9wxNqDp0qQzu3zKhtdKmSF/U5qGqfX7e7Z1/cQQAAAACMzSlVgJ6qOmbM22iyVXHnGIOf1KW7/yjuEAAAAADGZtzNAcVvq0v33G7SI3HnGINPdEyvvTHuEAAAAADGhgL0FGCS75/iN0l6Mu4shQoC3dt59gfOjzsHAAAAgMIl4g6A0th84EB2Vt/+751/+rTdMv2epKlxZ8pTjYLwA7P69n9ni+RxhwEAAACQP+aAnoKeW7iwuu/1wRvk1iBpbtx58mHy25ake+6NOwcAAACA/FGAnsJSUnD5zHlXyu0WSVdLqoo5Ui4Gs1lfuHxfz4txBwEAAACQHwpQSJKePvuDZ2WD4XqZrZD0EZVzMWrqmjytZvGirq6RuKMAAAAAyB0FKH7Lv507f2om41ea+VLJPuzSPJXZfGEz//KSPT1fijsHAAAAgNxRgGJUz5x54eRMTdWiULY4kH3I5ZdIOk9SdYyxMhb4ZUt+0/MfMWYAAAAAkAcKUBSkU0uqwvcfPE9ZzUmY5rg0x82mm4dTJJsiaapcU2SaImlCLuc0qd+lPkl9Mr0l9zdl1ueuvYFsr0u/8SDcK/M91Ur0fvS17f8X6ZsEAAAAUFQUoIjctpmLJ2aDwyctQve+dtGhem3OlioTAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACR+n+hWO+HMQy8KgAAAABJRU5ErkJggg=="

class Canvas(FigureCanvasQTAgg):
    def __init__(self, parent=None, figwidth=400, figheight=400, dpi=100):
        fig = plt.figure(figsize=(figwidth, figheight), dpi=dpi)
        self.axes = fig.add_subplot(111, xlim=(0.0, 1000.0), ylim=(0.0, 50.0))
        self.axes.set_title("pendulum_demo latency", fontsize=18)
        self.axes.set_xlabel("Count (1000Hz)", fontsize=12)
        self.axes.set_ylabel("Latency (us)", fontsize=12)
        super(Canvas, self).__init__(fig)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args=None, parent=None, width=600, height=800):
        super(MainWindow, self).__init__(parent)
        self.args = args
        self.set_logger()
        self.nproc = os.cpu_count()
        proc = subprocess.run(
            "which run", shell=True, capture_output=True)
        if proc.returncode:
            command = "taskset -c " + str(self.nproc-1)
        else:
            command = "run -b " + str(self.nproc-1)
        self.process_demo = QtCore.QProcess()
        self.process_demo.start(command + " pendulum_demo -i 0 -tp 90")
        self.process_test = QtCore.QProcess()
        self.process_test.readyRead.connect(self.on_readyRead_test)
        self.process_load = QtCore.QProcess()
        self.initUI(width, height)

    def closeEvent(self, event):
        if self.process_demo.state() != QtCore.QProcess.NotRunning:
            subprocess.run("killall logging", shell=True)
        if self.process_test.state() != QtCore.QProcess.NotRunning:
            self.process_test.terminate()
        if self.process_load.state() != QtCore.QProcess.NotRunning:
            self.process_load.terminate()
        time.sleep(1)
        event.accept()

    def set_logger(self):
        self.logger = getLogger(__name__)
        if self.args.debug:
            self.logger.setLevel(DEBUG)
        else:
            self.logger.setLevel(INFO)
        handler = StreamHandler()
        if self.args.debug:
            handler.setLevel(DEBUG)
        else:
            handler.setLevel(INFO)
        handler.setFormatter(Formatter(
            "%(asctime)s %(funcName)s [%(levelname)s]: %(message)s"))
        self.logger.addHandler(handler)

    def initUI(self, width, height):
        self.resize(width, height)
        self.setWindowTitle('pendulum_gui')
        # Layouts
        widget_top = QtWidgets.QWidget()
        vbox_top = QtWidgets.QVBoxLayout()
        hbox_sidebar = QtWidgets.QHBoxLayout()
        # Visualization
        self.canvas = Canvas(self);
        xdata = [i for i in range(0, 1000)]
        ydata = [None for i in range(0, 1000)]
        line2d = self.canvas.axes.plot(xdata, ydata, "b.", markersize=1)
        self.lines = line2d[0]
        vbox_top.addWidget(self.canvas)
        # Load Section
        gbox_load = QtWidgets.QGroupBox("System Load")
        form_load = QtWidgets.QFormLayout()
        form_load.setVerticalSpacing(15)
        form_load.setFormAlignment(QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.lineedit_usage = QtWidgets.QLineEdit()
        self.lineedit_usage.setMinimumWidth(120)
        self.lineedit_usage.setReadOnly(True)
        form_load.addRow("Load Avg:", self.lineedit_usage)
        self.button_load = QtWidgets.QPushButton("Stress")
        self.button_load.setCheckable(True)
        self.button_load.toggled.connect(self.on_toggled_load)
        form_load.addRow(self.button_load)
        gbox_load.setLayout(form_load)
        hbox_sidebar.addWidget(gbox_load, stretch=1)
        self.timer_load = QtCore.QTimer()
        self.timer_load.timeout.connect(self.on_timeout_load)
        self.timer_load.setInterval(500)
        self.timer_load.start()
        self.timer_usage = QtCore.QTimer()
        self.timer_usage.timeout.connect(self.on_timeout_usage)
        self.timer_usage.setInterval(500)
        self.timer_usage.start()
        # Test section
        gbox_test = QtWidgets.QGroupBox("Test")
        form_test = QtWidgets.QFormLayout()
        form_test.setVerticalSpacing(15)
        form_test.setFormAlignment(QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.lineedit_test = QtWidgets.QLineEdit()
        self.lineedit_test.setMinimumWidth(120)
        self.lineedit_test.setReadOnly(True)
        form_test.addRow("Max.(us):", self.lineedit_test)
        self.button_test = QtWidgets.QPushButton("Test")
        self.button_test.setCheckable(True)
        self.button_test.toggled.connect(self.on_toggled_test)
        form_test.addRow(self.button_test)
        gbox_test.setLayout(form_test)
        hbox_sidebar.addWidget(gbox_test, stretch=1)
        # Shield Section
        gbox_shield = QtWidgets.QGroupBox("Shield")
        vbox_shield = QtWidgets.QVBoxLayout()
        label_redhawk = QtWidgets.QLabel()
        label_redhawk.setAlignment(QtCore.Qt.AlignCenter)
        byte = QtCore.QByteArray.fromBase64(base64logo)
        pixmap = QtGui.QPixmap()
        pixmap.loadFromData(byte)
        pixmap = pixmap.scaledToWidth(200, QtCore.Qt.SmoothTransformation)
        label_redhawk.setPixmap(pixmap)
        vbox_shield.addWidget(label_redhawk)
        self.button_shield = QtWidgets.QPushButton("Shield")
        self.button_shield.setCheckable(True)
        self.button_shield.toggled.connect(self.on_toggled_shield)
        vbox_shield.addWidget(self.button_shield)
        gbox_shield.setLayout(vbox_shield)
        hbox_sidebar.addWidget(gbox_shield, stretch=1)
        vbox_top.addLayout(hbox_sidebar)
        widget_top.setLayout(vbox_top)
        self.setCentralWidget(widget_top)

    def on_toggled_load(self, checked):
        self.button_load.blockSignals(True)
        if checked:
            self.logger.debug("Load button switched ON.")
            proc = subprocess.run(
                "which stress-ng", shell=True, capture_output=True)
            if proc.returncode:
                message = "stress-ng isn't installed.\n" \
                          "Usually installable with following command.\n" \
                          "$ sudo apt install stress-ng"
                QtWidgets.QMessageBox.critical(self, "Error", message)
                self.button_load.setChecked(False)
            else:
                command = "stress-ng --cpu " + str(self.nproc)
                self.process_load.start(command)
        else:
            self.logger.debug("Load button switched OFF.")
            if self.process_load.state() != QtCore.QProcess.NotRunning:
                self.process_load.terminate()
        self.button_load.blockSignals(False)

    def on_timeout_load(self):
        proc = subprocess.run("ps aux|grep [s]tress-ng",
            shell=True, capture_output=True)
        if proc.returncode and self.button_load.isChecked():
            self.button_load.blockSignals(True)
            self.button_load.setChecked(False)
            self.button_load.blockSignals(False)
        elif not proc.returncode and not self.button_load.isChecked():
            self.button_load.blockSignals(True)
            self.button_load.setChecked(True)
            self.button_load.blockSignals(False)

    def on_timeout_usage(self):
        self.lineedit_usage.setText("%.2f, %.2f, %.2f" % os.getloadavg())

    def on_toggled_test(self, checked):
        if checked:
            self.logger.debug("Test button switched ON.")
            self.count = 0
            self.max = 0
            self.lineedit_test.setText("")
            ydata = [None for i in range(0, 1000)]
            self.lines.set_ydata(ydata)
            self.process_test.start("ros2 run pendulum_logging logging")
        else:
            self.logger.debug("Test button switched OFF.")
            if self.process_test.state() != QtCore.QProcess.NotRunning:
                self.process_test.terminate()

    def on_readyRead_test(self):
        while True:
            line = self.process_test.readLine()
            if line.isEmpty():
                break
            latency = line.toInt()[0]
            latency = float(latency) / 1000
            if latency > self.max:
                self.max = latency
                self.lineedit_test.setText(str(latency))
            ydata = self.lines.get_ydata()
            ydata[self.count] = latency
            self.lines.set_ydata(ydata)
            self.count += 1
            if self.count >= 1000:
                self.count = 0
        self.canvas.draw()

    def on_toggled_shield(self, checked):
        if checked:
            self.logger.debug("Shield button switched ON.")
            proc = subprocess.run("shield -c", shell=True, capture_output=True)
            if proc.returncode:
                message = "RedHawk Linux isn't installed.\n" \
                          "Or, running kernel isn't RedHawk Linux."
                QtWidgets.QMessageBox.critical(self, "Error", message)
                self.button_shield.blockSignals(True)
                self.button_shield.setChecked(False)
                self.button_shield.blockSignals(False)
            else:
                command = "pkexec shield -a +" + \
                          str(self.nproc - 2) + "-" + str(self.nproc - 1)
                subprocess.run(command, shell=True)
        else:
            self.logger.debug("Shield button switched OFF.")
            command = "pkexec shield -a -" + \
                      str(self.nproc - 2) + "-" + str(self.nproc - 1)
            subprocess.run(command, shell=True)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true")
    args = parser.parse_args()
    app = QtWidgets.QApplication([])
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
