#Author-AI
#Description-Generates a meshed pair of standard straight bevel gears (Pinion & Gear) at 90 degrees intersection.

import adsk.core, adsk.fusion, adsk.cam, traceback
import math

_handlers = []

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        cmdDef = ui.commandDefinitions.itemById('cmdBevelGearGenerator')
        if cmdDef:
            cmdDef.deleteMe()

        cmdDef = ui.commandDefinitions.addButtonDefinition('cmdBevelGearGenerator', '锥齿轮生成器 (90°)', '生成一对带有倒角、变位及间隙的直锥齿轮', '')

        onCommandCreated = BevelGearCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        _handlers.append(onCommandCreated)

        cmdDef.execute()
        adsk.autoTerminate(False)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def stop(context):
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        cmdDef = ui.commandDefinitions.itemById('cmdBevelGearGenerator')
        if cmdDef:
            cmdDef.deleteMe()
    except:
        pass

class BevelGearCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        try:
            cmd = args.command
            inputs = cmd.commandInputs
            
            # --- 创建UI ---
            inputs.addValueInput('module', '模数 (m)', 'mm', adsk.core.ValueInput.createByReal(2.0 / 10.0)) 
            inputs.addIntegerSpinnerCommandInput('teeth1', '小齿轮齿数 (z1)', 5, 500, 1, 15)
            inputs.addIntegerSpinnerCommandInput('teeth2', '大齿轮齿数 (z2)', 5, 500, 1, 30)
            inputs.addValueInput('faceWidth', '齿宽 (b)', 'mm', adsk.core.ValueInput.createByReal(12.0 / 10.0))
            inputs.addValueInput('pressureAngle', '压力角 (α)', 'deg', adsk.core.ValueInput.createByReal(math.radians(20.0)))
            
            inputs.addValueInput('shift1', '小齿轮变位 (x1)', '', adsk.core.ValueInput.createByReal(0.0))
            inputs.addValueInput('shift2', '大齿轮变位 (x2)', '', adsk.core.ValueInput.createByReal(0.0))
            
            inputs.addValueInput('backlash', '齿侧间隙 (j)', 'mm', adsk.core.ValueInput.createByReal(0.1 / 10.0))
            inputs.addValueInput('addendumFactor', '齿顶高系数 (ha*)', '', adsk.core.ValueInput.createByReal(1.0))
            inputs.addValueInput('clearanceFactor', '顶隙系数 (c*)', '', adsk.core.ValueInput.createByReal(0.25))
            inputs.addValueInput('rootFillet', '齿根倒角半径 (Rf)', 'mm', adsk.core.ValueInput.createByReal(0.3 / 10.0))

            onExecute = BevelGearCommandExecuteHandler()
            cmd.execute.add(onExecute)
            _handlers.append(onExecute)
        except:
            if adsk.core.Application.get().userInterface:
                adsk.core.Application.get().userInterface.messageBox('UI Error:\n{}'.format(traceback.format_exc()))

class BevelGearCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            ui  = app.userInterface
            design = app.activeProduct
            root_comp = design.rootComponent

            inputs = args.command.commandInputs

            # --- 获取参数 ---
            mod = inputs.itemById('module').value 
            z1 = inputs.itemById('teeth1').value
            z2 = inputs.itemById('teeth2').value
            b = inputs.itemById('faceWidth').value
            alpha = inputs.itemById('pressureAngle').value 
            x1 = inputs.itemById('shift1').value
            x2 = inputs.itemById('shift2').value
            backlash = inputs.itemById('backlash').value 
            ha_star = inputs.itemById('addendumFactor').value
            c_star = inputs.itemById('clearanceFactor').value
            root_fillet_r = inputs.itemById('rootFillet').value 

            # 计算分度锥角
            delta1 = math.atan(z1 / z2)
            delta2 = math.atan(z2 / z1)
            Re = mod * math.sqrt(z1**2 + z2**2) / 2.0

            if b >= Re:
                ui.messageBox('错误: 齿宽 (b) 不能大于或等于外锥距 (Re ≈ {:.2f} mm)'.format(Re*10))
                return

            # 创建齿轮组父组件
            occ_group = root_comp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            group_comp = occ_group.component
            group_comp.name = "Bevel Gear Set (m={})".format(mod*10)

            # 生成小齿轮 (沿-Z，啮合点翻转到下方以匹配)
            mat_pinion = adsk.core.Matrix3D.create()
            mat_pinion.setToRotation(math.pi, adsk.core.Vector3D.create(1,0,0), adsk.core.Point3D.create(0,0,0))
            self.create_single_bevel_gear(group_comp, "Pinion (z={})".format(z1), mat_pinion, mod, z1, delta1, Re, b, alpha, x1, backlash, ha_star, c_star, root_fillet_r)

            # 生成大齿轮 (旋转90度产生垂直啮合)
            mat_gear = adsk.core.Matrix3D.create()
            mat_gear.setToRotation(math.pi/2, adsk.core.Vector3D.create(0,1,0), adsk.core.Point3D.create(0,0,0))
            self.create_single_bevel_gear(group_comp, "Gear (z={})".format(z2), mat_gear, mod, z2, delta2, Re, b, alpha, x2, backlash, ha_star, c_star, root_fillet_r)

        except:
            if adsk.core.Application.get().userInterface:
                adsk.core.Application.get().userInterface.messageBox('Execute Failed:\n{}'.format(traceback.format_exc()))

    def create_single_bevel_gear(self, parent_comp, name, transform, m, z, delta, Re, b, alpha, x, backlash, ha_star, c_star, rf):
        occ = parent_comp.occurrences.addNewComponent(transform)
        comp = occ.component
        comp.name = name

        # 几何计算
        ha = m * (ha_star + x)
        hf = m * (ha_star + c_star - x)
        Ri = Re - b
        ratio = Ri / Re

        # 这里 _x 代表径向，_y 代表轴向高度
        Po_x, Po_y = Re * math.sin(delta), Re * math.cos(delta)
        
        To_x, To_y = Po_x + ha * math.cos(delta), Po_y - ha * math.sin(delta)
        Ro_x, Ro_y = Po_x - hf * math.cos(delta), Po_y + hf * math.sin(delta)
        
        Ti_x, Ti_y = Po_x * ratio + (ha * ratio) * math.cos(delta), Po_y * ratio - (ha * ratio) * math.sin(delta)
        Ri_x, Ri_y = Po_x * ratio - (hf * ratio) * math.cos(delta), Po_y * ratio + (hf * ratio) * math.sin(delta)

        # ==========================================
        # 1. 旋转基础毛坯实体 (恢复正确的 2D 坐标)
        # ==========================================
        sk_blank = comp.sketches.add(comp.xZConstructionPlane)
        lines = sk_blank.sketchCurves.sketchLines
        
        p1 = adsk.core.Point3D.create(0, Ro_y, 0)
        p2 = adsk.core.Point3D.create(Ro_x, Ro_y, 0)
        p3 = adsk.core.Point3D.create(To_x, To_y, 0)
        p4 = adsk.core.Point3D.create(Ti_x, Ti_y, 0)
        p5 = adsk.core.Point3D.create(Ri_x, Ri_y, 0)
        p6 = adsk.core.Point3D.create(0, Ri_y, 0)

        l1 = lines.addByTwoPoints(p1, p2)
        l2 = lines.addByTwoPoints(l1.endSketchPoint, p3)
        l3 = lines.addByTwoPoints(l2.endSketchPoint, p4)
        l4 = lines.addByTwoPoints(l3.endSketchPoint, p5)
        l5 = lines.addByTwoPoints(l4.endSketchPoint, p6)
        l6 = lines.addByTwoPoints(l5.endSketchPoint, l1.startSketchPoint)

        prof_blank = sk_blank.profiles.item(0)
        revolves = comp.features.revolveFeatures
        rev_input = revolves.createInput(prof_blank, comp.zConstructionAxis, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
        rev_input.setAngleExtent(False, adsk.core.ValueInput.createByReal(math.pi * 2))
        revolves.add(rev_input)

        # ==========================================
        # 2. 建立虚拟当量齿轮（切削刃）的切削平面
        # ==========================================
        pitch_line = lines.addByTwoPoints(adsk.core.Point3D.create(0,0,0), adsk.core.Point3D.create(Po_x, Po_y, 0))
        pitch_line.isConstruction = True
        
        path = comp.features.createPath(pitch_line)
        plane_input = comp.constructionPlanes.createInput()
        plane_input.setByDistanceOnPath(path, adsk.core.ValueInput.createByReal(1.0))
        cut_plane = comp.constructionPlanes.add(plane_input)

        # ==========================================
        # 3. 绘制虚拟直齿轮的“齿隙（Gap）”轮廓
        # ==========================================
        sk_cut = comp.sketches.add(cut_plane)
        sk_cut.isComputeDeferred = True

        zv = z / math.cos(delta)
        rv = Re * math.tan(delta)
        rbv = rv * math.cos(alpha)
        rav = rv + ha
        
        # 加深切削根部，防止零厚度报错
        rfv = rv - hf - (0.05 * m) 
        rext = rav + (0.5 * m) 

        # 【核心修复】：使用 API 自带的转换函数，精确将 2D 轮廓点转为 3D 空间点，再投影到切削平面上
        Po_3d = sk_blank.sketchToModelSpace(adsk.core.Point3D.create(Po_x, Po_y, 0))
        Cv_3d = sk_blank.sketchToModelSpace(adsk.core.Point3D.create(0, Re / math.cos(delta), 0))
        
        po_2d = sk_cut.modelToSketchSpace(Po_3d)
        cv_2d = sk_cut.modelToSketchSpace(Cv_3d)

        theta_sym = math.atan2(po_2d.y - cv_2d.y, po_2d.x - cv_2d.x)
        
        inv_alpha = math.tan(alpha) - alpha
        theta_t_half = (math.pi / (2 * zv)) + (2 * x * math.tan(alpha) / zv) - (backlash / (2 * rv))
        theta_g_half = (math.pi / zv) - theta_t_half

        pts_left, pts_right = [], []
        num_pts = 10
        
        for i in range(num_pts + 1):
            r = rfv + (rext - rfv) * (i / num_pts)
            
            if r <= rbv:
                theta_gap_r = theta_g_half - inv_alpha
            else:
                alpha_r = math.acos(rbv / r)
                inv_r = math.tan(alpha_r) - alpha_r
                theta_gap_r = theta_g_half + (inv_r - inv_alpha)

            angle_l = theta_sym + theta_gap_r
            pts_left.append(adsk.core.Point3D.create(cv_2d.x + r*math.cos(angle_l), cv_2d.y + r*math.sin(angle_l), 0))
            
            angle_r = theta_sym - theta_gap_r
            pts_right.append(adsk.core.Point3D.create(cv_2d.x + r*math.cos(angle_r), cv_2d.y + r*math.sin(angle_r), 0))

        splines = sk_cut.sketchCurves.sketchFittedSplines
        coll_l, coll_r = adsk.core.ObjectCollection.create(), adsk.core.ObjectCollection.create()
        for p in pts_left: coll_l.add(p)
        for p in pts_right: coll_r.add(p)
        
        spline_l = splines.add(coll_l)
        spline_r = splines.add(coll_r)

        cut_lines = sk_cut.sketchCurves.sketchLines
        cut_arcs = sk_cut.sketchCurves.sketchArcs
        
        top_line = cut_lines.addByTwoPoints(spline_l.endSketchPoint, spline_r.endSketchPoint)
        
        mid_root_pt = adsk.core.Point3D.create(cv_2d.x + rfv * math.cos(theta_sym), cv_2d.y + rfv * math.sin(theta_sym), 0)
        root_arc = cut_arcs.addByThreePoints(spline_r.startSketchPoint, mid_root_pt, spline_l.startSketchPoint)

        if rf > 0.001:
            try: cut_arcs.addFillet(spline_l, spline_l.startSketchPoint.geometry, root_arc, spline_l.startSketchPoint.geometry, rf)
            except: pass
            try: cut_arcs.addFillet(spline_r, spline_r.startSketchPoint.geometry, root_arc, spline_r.startSketchPoint.geometry, rf)
            except: pass

        sk_cut.isComputeDeferred = False
        
        prof_cut = None
        max_area = 0
        for i in range(sk_cut.profiles.count):
            prof = sk_cut.profiles.item(i)
            if prof.areaProperties().area > max_area:
                max_area = prof.areaProperties().area
                prof_cut = prof

        # ==========================================
        # 4. 放样切除 (Loft Cut) 至锥顶
        # ==========================================
        # 直接使用毛坯草图中的原点(pitch_line起点)作为放样尖端，极度稳定
        apex_pt = pitch_line.startSketchPoint

        lofts = comp.features.loftFeatures
        loft_input = lofts.createInput(adsk.fusion.FeatureOperations.CutFeatureOperation)
        loft_input.loftSections.add(prof_cut)
        loft_input.loftSections.add(apex_pt) 
        
        # 严格指定要切除的主实体
        if comp.bRepBodies.count > 0:
            loft_input.participantBodies = [comp.bRepBodies.item(0)]

        loft_feat = lofts.add(loft_input)

        # ==========================================
        # 5. 圆周阵列生成所有齿
        # ==========================================
        patterns = comp.features.circularPatternFeatures
        loft_coll = adsk.core.ObjectCollection.create()
        loft_coll.add(loft_feat)
        
        pat_input = patterns.createInput(loft_coll, comp.zConstructionAxis)
        pat_input.quantity = adsk.core.ValueInput.createByReal(z)
        # 高性能优化计算模式，防止布尔运算崩溃
        pat_input.patternComputeOption = adsk.fusion.PatternComputeOptions.OptimizedPatternCompute
        patterns.add(pat_input)

        # ==========================================
        # 6. 清理现场：自动隐藏草图和基准面
        # ==========================================
        for sk in comp.sketches:
            sk.isLightBulbOn = False  # 关闭草图的小眼睛
            
        for plane in comp.constructionPlanes:
            plane.isLightBulbOn = False  # 关闭构造平面的小眼睛