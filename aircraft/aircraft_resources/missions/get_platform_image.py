import cv2
import numpy as np


def generate_perfect_template(output_name="template_blueboat.png"):
    # 1. Cria uma imagem preta de 200x200 pixels (Tons de cinza)
    # Usaremos 200x200 para ter uma boa resolução de bordas
    size = 200
    img = np.zeros((size, size), dtype=np.uint8)

    # Definimos os tons de cinza baseados no contraste do mundo real:
    # No simulador, o fundo azul do quadrado fica escuro em cinza (~80)
    # O círculo e a cruz brancos ficam claros (~255)
    color_dark = 80
    color_light = 255

    # 2. Desenha o Quadrado Externo (Preenche a imagem quase toda)
    margin = 10
    cv2.rectangle(
        img, (margin, margin), (size - margin, size - margin), color_dark, -1
    )

    # 3. Desenha o Círculo Branco no centro
    center = (size // 2, size // 2)
    radius = 60
    cv2.circle(img, center, radius, color_light, -1)

    # 4. Desenha a Cruz Escura (dentro do círculo)
    # A cruz é feita de duas linhas (uma horizontal e outra vertical)
    cross_thickness = 14
    half_length = 45  # Comprimento dos braços da cruz

    # Linha Horizontal
    cv2.line(
        img,
        (center[0] - half_length, center[1]),
        (center[0] + half_length, center[1]),
        color_dark,
        cross_thickness,
    )

    # Linha Vertical
    cv2.line(
        img,
        (center[0], center[1] - half_length),
        (center[0], center[1] + half_length),
        color_dark,
        cross_thickness,
    )

    # 5. Aplica um leve desfoque para simular a suavização da câmera real
    final_template = cv2.GaussianBlur(img, (3, 3), 0)

    # 6. Salva a imagem
    cv2.imwrite(output_name, final_template)
    print(f"[SUCESSO] Template sintético gerado e salvo em: {output_name}")

    # Abre uma janela para você ver o resultado (pode fechar apertando qualquer tecla)
    cv2.imshow("Template Sintetico Gerado", final_template)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    generate_perfect_template()