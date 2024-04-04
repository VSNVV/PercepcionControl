% Generar un vector para el eje x (índices)
indice = 1:1000;
length(indice)
% Tomar solo el primer parámetro de cada conjunto de cinco para todos los elementos
primer_valor = medidas(1, :);
length(primer_valor)

% Graficar los valores del primer parámetro
plot(indice, primer_valor);
title('Evolución del Primer Parámetro de las Medidas');
xlabel('Índice');
ylabel('Primer Parámetro');