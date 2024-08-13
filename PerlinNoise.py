import pygame
import random
import numpy as np
from noise import pnoise2


class GenerateHeightMap:
    # Slider class to handle slider creation and interaction
    class Slider:
        def __init__(self, screen, x, y, w, h, min_val, max_val, start_val, name):
            self.screen = screen
            self.rect = pygame.Rect(x, y, w, h)
            self.min_val = min_val
            self.max_val = max_val
            self.val = start_val
            self.surf = pygame.Surface((w, h))
            self.surf.fill((100, 100, 100))
            self.handle = pygame.Rect(x, y, 10, h)
            self.handle_color = (200, 200, 200)
            self.dragging = False
            self.name = name

        def draw(self):
            pygame.draw.rect(self.screen, (100, 100, 100), self.rect)
            pygame.draw.rect(self.screen, self.handle_color, self.handle)
            font = pygame.font.Font(None, 20)
            text_surf = font.render(f"{self.name}: {self.val:.2f}", True, (255, 255, 255))
            self.screen.blit(text_surf, (self.rect.x, self.rect.y - 20))

        def update(self, event):
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.handle.collidepoint(event.pos):
                    self.dragging = True
            elif event.type == pygame.MOUSEBUTTONUP:
                self.dragging = False
            elif event.type == pygame.MOUSEMOTION:
                if self.dragging:
                    new_x = min(max(event.pos[0], self.rect.x), self.rect.x + self.rect.width)
                    self.handle.x = new_x
                    self.val = self.min_val + (self.max_val - self.min_val) * (
                                (self.handle.x - self.rect.x) / self.rect.width)

    class Noise:
        def __init__(self, map_width, map_height, seed, scale, octaves, persistence, lacunarity, offset):
            self.map_width = map_width
            self.map_height = map_height
            self.seed = seed
            self.scale = scale if scale > 0 else 0.0001
            self.octaves = octaves
            self.persistence = persistence
            self.lacunarity = lacunarity
            self.offset = offset

        def generate_noise_map(self):
            noise_map = np.zeros((self.map_width, self.map_height))
            prng = random.Random(self.seed)
            octave_offsets = [(
                prng.randint(-100000, 100000) + self.offset[0],
                prng.randint(-100000, 100000) + self.offset[1]
            ) for _ in range(self.octaves)]

            max_noise_height = float('-inf')
            min_noise_height = float('inf')

            half_width = self.map_width / 2.0
            half_height = self.map_height / 2.0

            for y in range(self.map_height):
                for x in range(self.map_width):
                    amplitude = 1
                    frequency = 1
                    noise_height = 0

                    for i in range(self.octaves):
                        sample_x = (x - half_width) / self.scale * frequency + octave_offsets[i][0]
                        sample_y = (y - half_height) / self.scale * frequency + octave_offsets[i][1]

                        perlin_value = pnoise2(sample_x, sample_y) *2 - 1
                        noise_height += perlin_value * amplitude

                        amplitude *= self.persistence
                        frequency *= self.lacunarity

                    max_noise_height = max(max_noise_height, noise_height)
                    min_noise_height = min(min_noise_height, noise_height)
                    noise_map[x, y] = noise_height

            for y in range(self.map_height):
                for x in range(self.map_width):
                    noise_map[x, y] = self.inverse_lerp(min_noise_height, max_noise_height, noise_map[x, y])

            return noise_map

        @staticmethod
        def inverse_lerp(a, b, value):
            return (value - a) / (b - a)

    def __init__(self):
        self.map_width = 100
        self.map_height = 100
        self.seed = 42
        self.scale = 50.0
        self.octaves = 6
        self.persistence = 0.5
        self.lacunarity = 2.0
        self.offset = (0, 0)

    def generate(self):

        pygame.init()
        self.screen = pygame.display.set_mode((800, 600))
        pygame.display.set_caption("Perlin Noise Map Generator")

        self.sliders = [
            self.Slider(self.screen, 550, 100, 200, 20, 10, 400, self.map_width, 'Width'),
            self.Slider(self.screen, 550, 150, 200, 20, 10, 400, self.map_height, 'Height'),
            self.Slider(self.screen, 550, 200, 200, 20, 50, 100, self.seed, 'Seed'),
            self.Slider(self.screen, 550, 250, 200, 20, 50, 100, self.scale, 'Scale'),
            self.Slider(self.screen, 550, 300, 200, 20, 1, 10, self.octaves, 'Octaves'),
            self.Slider(self.screen, 550, 350, 200, 20, 0, 1, self.persistence, 'Persistence'),
            self.Slider(self.screen, 550, 400, 200, 20, 1, 4, self.lacunarity, 'Lacunarity')
        ]

        new_noise_map = []
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                for slider in self.sliders:
                    slider.update(event)

            self.screen.fill((255, 255, 255))

            new_noise_map = self.regenerate_noise_map()
            noise_surface = self.draw_noise_map(new_noise_map)

            self.screen.blit(noise_surface, (0, 0))

            for slider in self.sliders:
                slider.draw()

            pygame.display.flip()

        pygame.quit()

        if len(new_noise_map) > 0:
            # After quitting the game, generate the height map
            height_map = new_noise_map *  40
            # # Optionally, save the height map to a file
            # np.savetxt("height_map.txt", height_map, fmt='%.2f')

            return height_map, self.createVoxelGrid(height_map), self.computeslope(height_map)

    def computeslope(self, noiseMap):
        slope = np.zeros((noiseMap.shape[0], noiseMap.shape[1]))
        for i in range(noiseMap.shape[0]):
            for j in range(noiseMap.shape[1]):
                elevation_angle = np.arctan2(noiseMap[i,j], np.sqrt(i ** 2 + j ** 2))
                elevation_angle_degree = elevation_angle*(180/np.pi)
                slope[i, j] = elevation_angle_degree

        return slope

    def createVoxelGrid(self, noiseMap):

        map_data = np.array([(x, y, noiseMap[x, y]) for x in range(noiseMap.shape[0]) for y in range(noiseMap.shape[1])])
        x_min, x_max, y_min, y_max, z_min, z_max = 0, noiseMap.shape[0], 0, noiseMap.shape[1], np.min(noiseMap), np.max(noiseMap)
        x_grid = np.arange(x_min, x_max, 1)
        y_grid = np.arange(y_min, y_max, 1)
        z_grid = np.arange(z_min, z_max, 1)
        X, Y, Z = np.meshgrid(x_grid, y_grid, z_grid, indexing='ij')
        grid = np.zeros_like(X)

        for point in map_data:
            x, y, z = point

            if x_min <= x < x_max and y_min <= y < y_max and z_min <= z < z_max:
                i = int((x - x_min) )
                j = int((y - y_min) )
                k = int((z - z_min))
                grid[i, j, k] = 1

        return grid

    def regenerate_noise_map(self):
        self.map_width = int(self.sliders[0].val)
        self.map_height = int(self.sliders[1].val)
        self.seed = int(self.sliders[2].val)
        self.scale = self.sliders[3].val
        self.octaves = int(self.sliders[4].val)
        self.persistence = self.sliders[5].val
        self.lacunarity = self.sliders[6].val

        noise = self.Noise(self.map_width, self.map_height, self.seed, self.scale,
                           self.octaves, self.persistence, self.lacunarity, self.offset)
        return noise.generate_noise_map()

    def draw_noise_map(self, noise_map):
        noise_surface = pygame.Surface((self.map_width, self.map_height))
        for y in range(self.map_height):
            for x in range(self.map_width):
                color = int(noise_map[x, y] * 255)
                noise_surface.set_at((x, y), (color, color, color))
        return noise_surface


# if __name__ == "__main__":
#     GenerateHeightMap()
