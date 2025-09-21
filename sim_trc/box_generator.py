import os
import shutil

# Dossier contenant les images source
IMAGE_DIR = "./images"
# Dossier de sortie pour les modèles Gazebo
OUTPUT_DIR = os.path.expanduser("./models")

# Template du fichier SDF
SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>true</static>
    <link name="link">
      <visual name="cube_visual">
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size>
          </box>
        </geometry>
        <material>
          <script>
            <uri>model://{model_name}/materials/scripts/</uri>
            <uri>model://{model_name}/materials/textures/</uri>
            <name>{model_name}/FaceMaterial</name>
          </script>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.03 0.03 0.03</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

# Template du fichier model.config
CONFIG_TEMPLATE = """<?xml version="1.0" ?>
<model>
  <name>{model_name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>BoxGenerator</name>
    <email>charifou.mousse@tekbot.io</email>
  </author>
  <description>Cube avec texture {model_name}</description>
</model>
"""

# Template du fichier .material
MATERIAL_TEMPLATE = """material {model_name}/FaceMaterial
{{
  technique
  {{
    pass
    {{
      texture_unit
      {{
        texture {image_name}
      }}
    }}
  }}
}}
"""

def create_cube_model(image_path):
    image_name = os.path.basename(image_path)
    model_name = os.path.splitext(image_name)[0]

    model_dir = os.path.join(OUTPUT_DIR, model_name)
    materials_scripts = os.path.join(model_dir, "materials", "scripts")
    materials_textures = os.path.join(model_dir, "materials", "textures")

    # Créer l'arborescence
    os.makedirs(materials_scripts, exist_ok=True)
    os.makedirs(materials_textures, exist_ok=True)

    # Copier l'image dans le dossier textures
    shutil.copy(image_path, os.path.join(materials_textures, image_name))

    # Créer model.sdf
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(SDF_TEMPLATE.format(model_name=model_name))

    # Créer model.config
    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(CONFIG_TEMPLATE.format(model_name=model_name))

    # Créer le fichier material
    with open(os.path.join(materials_scripts, f"{model_name}.material"), "w") as f:
        f.write(MATERIAL_TEMPLATE.format(model_name=model_name, image_name=image_name))

    print(f"✅ Modèle généré : {model_name}")


def main():
    if not os.path.exists(IMAGE_DIR):
        print(f"❌ Le dossier {IMAGE_DIR} n'existe pas.")
        return

    images = [f for f in os.listdir(IMAGE_DIR) if f.lower().endswith((".png", ".jpg", ".jpeg"))]

    if not images:
        print("❌ Aucune image trouvée dans le dossier.")
        return

    for img in images:
        create_cube_model(os.path.join(IMAGE_DIR, img))


if __name__ == "__main__":
    main()
